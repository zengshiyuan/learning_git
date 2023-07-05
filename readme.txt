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

git reset HEAD <file>可以把暂存区的修改撤销掉（unstage），重新放回工作区

git rm <file>,从仓库中删除该文件

关联远程库
git remote add origin git@github.com:github_usename/learngit.git  

第一次推送master分支的所有内容，所以有“-u”
git push -u origin master
只要本地作了提交，就可以通过命令把本地master分支的最新修改推送至GitHub
git push origin master  

添加的时候地址写错了，或者就是想删除远程库，可以用git remote rm <name>命令。
使用前，建议先用git remote -v查看远程库信息：
比如删除origin：git remote rm origin


"""""""""""
查看分支：git branch

创建分支：git branch <name>

切换分支：git checkout <name>或者git switch <name>

创建+切换分支：git checkout -b <name>或者git switch -c <name>

合并某分支到当前分支：git merge <name>

删除分支：git branch -d <filename>
""""""""""

git log --graph命令可以看到分支合并图。

把当前工作现场“储藏”起来，等以后恢复现场后继续工作：
git stash

查看远程库的信息
git remote

显示仓库的地址和路径
git remote -v


rebase操作可以把本地未push的分叉提交历史整理成直线；
rebase的目的是使得我们在查看历史提交的变化时更容易，因为分叉的提交需要三方对比。
git rebase

在Git中打标签,首先，切换到需要打标签的分支上
git tag <name>
git tag v1.0

查看所有标签
git tag

查找历史commit id
git log --pretty=oneline --abbrev-commit

给某个commit id打上标签
git tag <标签名> commit_id。

查看标签信息：
git show <tagname>

创建带有说明的标签，用-a指定标签名，-m指定说明文字
git tag -a v0.1 -m "说明文字" 1094adb

删除标签
git tag -d <tagname>

推送某个标签到远程
git push origin <tagname>
推送全部标签
git push origin --tags

命令git tag -d <tagname>可以删除一个本地标签；

命令git push origin :refs/tags/<tagname>可以删除一个远程标签。
