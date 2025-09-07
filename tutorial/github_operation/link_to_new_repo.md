# 链接本地仓库到github

## 1. 准备好本地仓库
创建本地代码，比如`mkdir HitchOpen-THICV-Stack`，进入目录`cd HitchOpen-THICV-Stack`。里面该写的代码啥的都写好就行

## 1. 在github上创建一个新的仓库
1.登录Github账号，点击右上角`+ → New repository`创建一个新的仓库
2. 填写仓库信息：
- **Repository name**：`HitchOpen-THICV-Stack`，一定要和本地仓库名称一致（最好，否则会很乱）
- **Description**：可选，仓库的描述
- **Visibility**：根据需要选择仓库的可见性。
- **Initialize this repository with a README**：如果是本地直接创建好仓库的话建议不要勾选，否则后面会遇到合并仓库等等麻烦的事情，尽量保持仓库为空的或者至少不要与本地聂荣冲突。
- **Add a .gitignore**：根据需要添加`.gitignore`文件，比如`catkin`、`python`、`ros`等这些已经写好的内容。
- **Add a license**：根据需要添加许可证，比如`MIT`、`Apache 2.0`等。

3. 点击`Create repository`创建仓库，并复制仓库的URL（有2种）：
- **HTTPS**：`https://github.com/KomasaQi/HitchOpen-THICV-Stack.git` (推荐，简单方便)
- **SSH**：`git@github.com:KomasaQi/HitchOpen-THICV-Stack.git` (需要配置SSH密钥，比较麻烦)

## 2. 链接本地仓库到github

打开终端，进入本地仓库的目录，比如

``` bash
# 1.进入本地仓库目录
cd ~/HitchOpen-THICV-Stack

# 2.初始化git仓库
git init

# 3. 查看当前文件状态（确认忽略文件生效，红色为未跟踪，绿色为已跟踪）
git status

# 4. 修改本地默认分支为main
git branch -M main

# 5. 添加远程仓库
git remote add origin https://github.com/KomasaQi/HitchOpen-THICV-Stack.git

# 6. 验证远程仓库关联是否正确（显示 fetch/push 链接即正常）
git remote -v

# 拉取远程 main 分支，并合并无关历史（首次推送常见问题“远程仓库已存在文件冲突，这在Github仓库初始化时已经生成README.md或者LICENSE或者.gitignore文件的时候就需要本地先和远程同步一下”）
git pull origin main --allow-unrelated-histories

# 7. 将所有文件添加到 Git 暂存区（. 表示当前目录所有文件）
git add .

# 8. 提交暂存区文件到本地仓库（-m 后写提交说明，清晰描述内容）
git commit -m "Initial commit"


# 9. 推送本地代码到 GitHub 远程仓库
# -u origin main：设置默认推送分支为 main（GitHub 新仓库默认分支名）
git push -u origin main


```

