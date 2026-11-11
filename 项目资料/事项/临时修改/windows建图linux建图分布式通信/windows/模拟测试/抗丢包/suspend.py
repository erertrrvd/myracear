"""suspend.py <pid> <seconds>：挂起进程若干秒再恢复（模拟电脑端收不到数据）。"""
import ctypes
import sys
import time

pid, sec = int(sys.argv[1]), float(sys.argv[2])
k = ctypes.windll.kernel32
nt = ctypes.windll.ntdll
h = k.OpenProcess(0x0800, False, pid)   # PROCESS_SUSPEND_RESUME
if not h:
    sys.exit(f'OpenProcess failed for {pid}')
nt.NtSuspendProcess(h)
print(f'suspended {pid}', flush=True)
time.sleep(sec)
nt.NtResumeProcess(h)
print(f'resumed {pid}', flush=True)
