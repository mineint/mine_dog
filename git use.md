一、初始化阶段（首次将本地代码推送到远程空仓库）
bash

# 1. 进入本地代码目录
cd mine_dog

# 2. 初始化 Git 仓库（如果还没有）
git init

# 3. 关联远程仓库（替换为你的实际地址）
git remote add origin git@github.com:mineint/mine_dog.git

# 4. 添加所有文件并首次提交
git add .
git commit -m "Initial commit"

# 5. 推送到远程（主分支可能是 main 或 master）
git push -u origin main       # 或 git push -u origin master

    如果远程仓库已存在其他文件（如 README），先拉取合并：
    bash

    git pull origin main --allow-unrelated-histories
    # 解决冲突后
    git push origin main

二、日常协作标准流程（每天使用）
bash

# 1. 开始工作前：拉取队友的最新代码
git pull origin main

# 2. 修改代码（用编辑器完成）

# 3. 查看改动状态
git status

# 4. 添加改动到暂存区
git add .                     # 添加所有改动
# 或 git add <具体文件名>

# 5. 提交到本地仓库
git commit -m "清晰描述你的改动，如：修复登录按钮样式"

# 6. 再次拉取（避免推送时冲突）
git pull origin main

# 7. 推送到远程
git push origin main

彻底放弃本地修改，完全以云端为准：
git reset --hard HEAD
git pull origin main



如果你想保留你刚才写的代码，同时也想把服务器上的新代码拉下来。
先暂存本地修改：
code Bash
git stash
这会把你的代码暂时“藏起来”，让工作区恢复到干净状态。
拉取最新代码：
code Bash
git pull origin main
恢复你的修改：
code Bash
git stash pop

三、处理合并冲突（当 git pull 提示冲突时）
bash

# 1. 查看冲突文件
git status

# 2. 手动编辑冲突文件（删除 <<<<<<<, =======, >>>>>>> 标记，保留最终代码）

# 3. 标记为已解决
git add <冲突文件名>

# 4. 完成合并提交
git commit -m "解决合并冲突"

# 5. 推送
git push origin main

四、常用辅助命令
命令	说明
git status	查看工作区状态（哪些文件被修改/未跟踪）
git diff	查看未暂存的具体修改内容
git log --oneline --graph	查看简洁的提交历史树
git branch	查看本地分支
git checkout -b <新分支名>	创建并切换到新分支（用于开发新功能）
git remote -v	查看远程仓库地址
ssh -T git@github.com	测试 SSH 连接是否正常
五、在新电脑上使用同一个账号
bash

# 1. 配置用户信息
git config --global user.name "你的GitHub用户名"
git config --global user.email "你的GitHub邮箱"

# 2. 生成 SSH 密钥（一路回车）
ssh-keygen -t ed25519 -C "你的GitHub邮箱"

# 3. 复制公钥并添加到 GitHub（Settings → SSH and GPG keys）
cat ~/.ssh/id_ed25519.pub

# 4. 测试连接
ssh -T git@github.com

# 5. 克隆项目
git clone git@github.com:mineint/mine_dog.git

# 6. 进入目录开始日常协作
cd mine_dog

核心原则

    推送前先拉取：git pull → 解决冲突 → git push

    小步提交，频繁同步：减少冲突概率

    写清晰的提交信息：方便回溯

    不要在主分支直接开发大功能（可选）：用 feature 分支 + Pull Request

