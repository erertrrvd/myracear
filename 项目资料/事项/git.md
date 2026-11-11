在 Windows PowerShell 中，先按 Ctrl+C 退出当前的密码提示，然后逐行运行：
cd 'C:\Users\lzc2\Desktop\my_racecar（复件）'

git stash push -u -m "before merging car remote"
git fetch origin
git merge --no-edit origin/main
git push origin main
git stash pop --index
fetch 和 push 提示时，在终端输入小车的 SSH 密码。stash 是为了临时保管你已暂存的另一份文档；最后一条命令会恢复它。如果 fetch 失败或 merge 报冲突，就停在那一步，不要继续执行后面的命令，也不要使用 --force。
推送成功后，还需在小车终端更新实际运行的代码；推送到裸仓库本身不会更新 /home/racecar/src/my_racecar：
cd /home/racecar/src/my_racecar
git status --short --branch
git fetch origin
git merge origin/main
车上若报告未提交改动或合并冲突，也先停下处理，之后再编译。



同步
cd 'C:\Users\lzc2\Desktop\my_racecar（复件）'
git remote add car ssh://root@192.168.5.100/home/racecar/src/my_racecar
git fetch car
git merge car/main