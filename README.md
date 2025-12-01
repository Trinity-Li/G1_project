# 天津大学具身智能社团官方文档
**这是一个对该社团所有相关技术文档的汇总，亦是对利用Github进行项目管理的尝试**
### 情况一：标准流程（Fork & Pull Request）

**适用对象：** 任何没有你仓库密码、不是你队友的人（或者是你想在合并代码前进行审核的人）。

这是最安全、最标准的 GitHub 协作方式。其他人无法直接修改你的仓库，只能先复制一份，改完后请求你合并。

#### 1. 对方的操作步骤（贡献者视角）

1. **Fork（复刻）：**
    
    - 对方打开你的链接：`https://github.com/Trinity-Li/G1_project`
        
    - 点击右上角的 **Fork** 按钮。
        
    - **结果：** 对方的账号下会多出一个 `G1_project` 仓库（例如 `https://github.com/张三/G1_project`）。
        
2. **Clone（下载副本）：**
    
    - 对方在**他们自己的**电脑上终端输入：
        
    
    Bash
    
    ```
    # 注意：这里克隆的是“张三”自己的库，而不是 Trinity-Li 的库
    git clone https://github.com/张三/G1_project.git
    cd G1_project
    ```
    
3. **修改与提交：**
    
    - 对方新建一个分支（好习惯）：
        
    
    Bash
    
    ```
    git checkout -b new-feature
    ```
    
    - 修改代码，然后保存：
        
    
    Bash
    
    ```
    git add .
    git commit -m "添加了机械臂抓取功能"
    ```
    
4. **Push（推送到他们的仓库）：**
    
    Bash
    
    ```
    git push origin new-feature
    ```
    
5. **发起 PR（Pull Request）：**
    
    - 对方回到**你的** GitHub 页面（或者他们自己的页面，GitHub 会自动提示）。
        
    - 点击 **Compare & pull request**。
        
    - 填写说明：“我修复了一个Bug，请合并。” -> 点击 **Create Pull Request**。
        

#### 2. 你的操作步骤（仓库主人视角）

1. 你会收到一封邮件通知，或者在仓库的 **Pull requests** 栏看到一个红色的 `(1)`。
    
2. 点进去查看他的代码变动。
    
3. 如果没问题，点击绿色的 **Merge pull request** 按钮。
    
4. **结果：** 他的代码正式进入了你的 `Trinity-Li/G1_project` 仓库。
    

---

### 情况二：队友模式（Collaborator）

**适用对象：** 跟你一起做大作业的同学、高度信任的合作伙伴。

如果你不想每次都审核 PR，可以直接给他们“家门钥匙”。

#### 1. 你的操作（设置权限）

1. 进入仓库页面 -> **Settings** -> **Collaborators**。
    
2. 点击 **Add people**，输入对方的 GitHub 用户名或邮箱。
    
3. 对方在邮箱里接受邀请。
    

#### 2. 对方的操作步骤（队友视角）

他们**不需要 Fork**，可以直接对你的仓库进行读写。

1. **Clone（直接下载你的库）：**
    
    Bash
    
    ```
    # 直接克隆你的源仓库
    git clone https://github.com/Trinity-Li/G1_project.git
    ```
    
2. **修改与推送：**
    
    - 修改代码。
        
    - 直接推送到你的仓库：
        
    
    Bash
    
    ```
    git add .
    git commit -m "更新了控制算法"
    git push origin main
    ```
    
    _(注意：如果你们两个人同时改了同一个文件，这一步可能会提示冲突，需要先 `git pull` 解决冲突再推)_。