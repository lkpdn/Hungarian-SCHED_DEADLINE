#!/usr/bin/env python3

import glob
import logging
import math
import os
import re
import sys
import subprocess
from tempfile import NamedTemporaryFile
from enum import Enum
from pulp import LpVariable, LpProblem, lpSum, lpDot, value, PULP_CBC_CMD

logging.basicConfig(format="%(levelname)s:%(message)s", level=logging.DEBUG)

SCHED_FIXEDPOINT_SCALE = 1 << 10
SYSFS_CPU = "/sys/devices/system/cpu/"


class ProcModel(Enum):
    IG = 1
    IA = 2
    UG = 3
    UA = 4


class DlTask:
    def __init__(self, pid, affinity, runtime, deadline, period):
        self.pid = pid
        self.affinity = affinity
        self.runtime = runtime
        self.deadline = deadline
        self.period = period

    def density(self):
        return self.runtime / self.deadline

    def utilization(self):
        return self.runtime / self.period

    def is_implicit(self):
        return self.deadline == self.period


def get_cpu_capacity():
    cpu_caps = []
    for i in range(os.cpu_count()):
        try:
            with open(f"{SYSFS_CPU}/cpu{i}/cpu_capacity", "r") as f:
                cpu_caps.append(int(f.read().rstrip()) / SCHED_FIXEDPOINT_SCALE)
        except FileNotFoundError:
            cpu_caps.append(0.0)
        except Exception as e:
            logging.error(f"cannot get cpu_capacity for cpu#{i}: {e}")
    if sum(cpu_caps) == 0.0:
        cpu_caps = [1.0 for _ in range(os.cpu_count())]
    return cpu_caps


def get_task_info():
    dl_tasks = []
    dl_pids = []
    for p_stat in glob.glob("/proc/[0-9]*/task/[0-9]*/stat"):
        with open(p_stat, "r") as f:
            fields = f.read().rstrip().split()
            if int(fields[40]) == 6:  # SCHED_DEADLINE
                dl_pids.append(int(fields[0]))
    for dl_pid in dl_pids:
        res = subprocess.run(
            ["chrt", "-p", str(dl_pid)], stdout=subprocess.PIPE
        ).stdout.decode("utf-8")
        m = re.match(
            "pid.*runtime/deadline/period parameters: (\d+)/(\d+)/(\d+)",
            res.split("\n")[-2],
        )
        if not m:
            continue
        dl_runtime = int(m.groups()[0])
        dl_deadline = int(m.groups()[1])
        dl_period = int(m.groups()[2])
        with open(f"/proc/{dl_pid}/status", "r") as f:
            for line in f.readlines():
                if "Cpus_allowed:" in line:
                    cpu_affinity = int(line.split()[-1], base=16)
        dl_task = DlTask(dl_pid, cpu_affinity, dl_runtime, dl_deadline, dl_period)
        dl_tasks.append(dl_task)
    return dl_tasks


def get_root_domains():
    n = os.cpu_count()

    # TODO: replace this with a light-weight, clean solution.
    if not os.path.exists("/proc/kcore"):
        logging.warning(
            "Beware that the result would be bogus if multiple "
            + "SCHED_DEADLINE tasks span multiple root domains"
        )
        return ["<default>"] * n

    f = NamedTemporaryFile()
    fname = f.name
    f.write(b"set $i = 0\n")
    f.write(f"while ($i < {n})\n".encode())
    f.write(
        b"p ((struct rq *)((unsigned long)&runqueues + \
            (unsigned long)__per_cpu_offset[$i++]))->rd\n"
    )
    f.write(b"end\n")
    f.write(b"quit\n")
    f.flush()
    cmd = f"gdb -q /lib/modules/`uname -r`/build/vmlinux -x {fname} /proc/kcore"
    try:
        rds = []
        result = subprocess.run(
            cmd, shell=True, text=True, capture_output=True, timeout=10
        )
        for line in result.stdout.split("\n"):
            if "root_domain" in str(line):
                m = re.match(r".* (0x[0-9a-z]*).*", str(line))
                if m:
                    rds.append(m.groups()[0])
        if len(rds) == n:
            return rds
    except subprocess.TimeoutExpired:
        pass

    logging.error("Failed to get root domains.")
    logging.warning(
        "Beware that the result would be bogus if multiple "
        + "SCHED_DEADLINE tasks span multiple root domains"
    )
    return ["<default>"] * n


def get_rd_cpumasks():
    res = {}
    rds = get_root_domains()
    for cpu in range(os.cpu_count()):
        rd = rds[cpu]
        if rd not in res:
            res[rd] = 1 << cpu
        else:
            res[rd] |= 1 << cpu
    return list(res.values())


def get_processor_model(cpu_caps, dl_tasks, rd_cpumask):
    is_identical = False
    is_global = False
    if len(set([cap for i, cap in enumerate(cpu_caps) if i & rd_cpumask])) == 1:
        is_identical = True
    if len(list(filter(lambda p: p.affinity < rd_cpumask, dl_tasks))) == 0:
        is_global = True
    if is_identical and is_global:
        return ProcModel.IG
    elif is_identical:
        return ProcModel.IA
    elif is_global:
        return ProcModel.UG
    else:
        return ProcModel.UA


def process_ua(cpu_caps, dl_tasks, rd_cpumask):
    for p in dl_tasks:
        if not p.is_implicit():
            logging.warning(
                f"task {p.pid} is not an implicit-deadline task. "
                + "will do pessimistic density-based evaluation"
            )

    # 1. get 'l' value (this also serves as feasibility checking)
    n = max(len(dl_tasks), len(cpu_caps))
    s = []  # speed
    for i in range(n):
        si = []
        if i < len(dl_tasks):
            affine = dl_tasks[i].affinity
            for j in range(n):
                if j & affine and j & rd_cpumask:
                    si.append(cpu_caps[j])
                else:
                    si.append(0)
        else:
            si = [0 for _ in range(n)]
        s.append(si)

    l = LpVariable("l", lowBound=0, upBound=1)
    x = []
    for i in range(n):
        xi = []
        for j in range(n):
            lpv = LpVariable(f"x{i}{j}", lowBound=0, upBound=1)
            xi.append(lpv)
        x.append(xi)

    prob = LpProblem()
    prob += -l
    for i in range(n):
        ui = 0
        if i < len(dl_tasks):
            # WARNING: set density, instead of utilization. Originally,
            # only implicit-deadline task model is supported.
            ui = dl_tasks[i].density()
        prob += lpSum(x[i][j] for j in range(n)) == 1 - l
        prob += lpDot(s[i], x[i]) >= ui
    prob.solve(PULP_CBC_CMD(msg=0))
    max_l = value(l)
    logging.debug(f"max_l = {max_l}")
    if max_l == 0:
        logging.warning("Tight so tardiness may be unbounded.")
        return

    # 2. calcuate tardiness bound for each task
    t_max = max([p.period for p in dl_tasks])
    s_max = max(cpu_caps)
    u_max = max([p.density() for p in dl_tasks])
    u_min = min([p.density() for p in dl_tasks])  # note: u==0 skipped.
    factor = (2 * n * t_max * s_max) / (max_l * u_min)
    logging.debug(
        f"t_max = {t_max}, s_max = {s_max}, u_max = {u_max}, u_min = {u_min}, n = {n}"
    )
    for i in range(len(dl_tasks)):
        ui = dl_tasks[i].density()
        tardiness = math.sqrt(u_max / ui) * factor
        print(f"tardiness bound for task#{i} (pid = {dl_tasks[i].pid}): {tardiness} ns")


def process_hp_lag_compliant(cpu_caps, dl_tasks, rd_cpumask):
    for p in dl_tasks:
        if not p.is_implicit():
            logging.warning(
                f"task {p.pid} is not an implicit-deadline task. "
                + "will do pessimistic density-based evaluation"
            )

    t_max = max([p.period for p in dl_tasks])
    u_min = min([p.density() for p in dl_tasks])
    u_sum = sum([p.density() for p in dl_tasks])

    logging.debug(f"t_max = {t_max}, u_min = {u_min}, u_sum = {u_sum}")
    for i in range(len(dl_tasks)):
        ui = dl_tasks[i].density()
        tardiness = (t_max / (2 * u_min)) * (2 * u_sum - ui)
        print(f"tardiness bound for task#{i} (pid = {dl_tasks[i].pid}): {tardiness} ns")


def process_ig(cpu_caps, dl_tasks, rd_cpumask):
    # TODO
    # 1). harmonic bound
    # 2). exact bound for pseudo-harmonic tasks case
    print("unimplemented (TODO)")


def process(cpu_caps, dl_tasks, rd_cpumask):
    f_dl_tasks = list(filter(lambda p: p.affinity & rd_cpumask, dl_tasks))
    if len(f_dl_tasks) == 0:
        return

    proc_model = get_processor_model(cpu_caps, f_dl_tasks, rd_cpumask)
    if proc_model == ProcModel.UA:
        process_ua(cpu_caps, f_dl_tasks, rd_cpumask)
    elif proc_model == ProcModel.UG or proc_model == ProcModel.IA:
        process_hp_lag_compliant(cpu_caps, dl_tasks, rd_cpumask)
    else:
        process_ig(cpu_caps, f_dl_tasks, rd_cpumask)


if __name__ == "__main__":
    cpu_caps = get_cpu_capacity()
    dl_tasks = get_task_info()
    rd_cpumasks = get_rd_cpumasks()
    for rd_cpumask in rd_cpumasks:
        process(cpu_caps, dl_tasks, rd_cpumask)
