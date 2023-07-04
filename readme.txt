Git is a version control system.
Git is free software.


初始化一个Git仓库，使用git init命令。

添加文件到Git仓库，分两步：

    使用命令git add <file>，注意，可反复多次使用，添加多个文件；
    使用命令git commit -m "message"，完成。

git status,查看仓库当前状态
git diff <file>，查看文件上一次的更新信息

git log命令显示从最近到最远的提交日志，

git reset --hard HEAD^ ,版本回退
HEAD指向的版本就是当前版本，因此，Git允许我们在版本的历史之间穿梭
使用命令git reset --hard commit_id。

git reflog用来记录你的每一次命令：

git checkout -- readme.txt意思就是，把readme.txt文件在工作区的修改全部撤销