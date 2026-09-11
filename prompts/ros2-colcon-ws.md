# ROS 2 colcon_ws rule
- When you build the workspace, execute `colcon build` under the toplevel of the colcon workspace
- When you build the workspace, please execute `colcon build --symlink-install <other options>`
- For each repositories, please execute the lint checking
- Do not execute git commands without the user instruction
- If you need to execute `apt install` command, please ask the user to do it
- The appropriate environment variables are set using commands such as `source`, etc
- Since the appropriate environment variables are set by the user, unless otherwise specified, `source` or `DOMAIN_ID`
- If the changes result in unnecessary code, comments or dependencies, please remove them
