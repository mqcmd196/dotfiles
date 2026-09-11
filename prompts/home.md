# Toplevel rule
- Write comments in the code in English, keeping them short and concise
- Git commits, pushes, and the creation of pull requests should only be performed after the user has given permission
- Unless you are using a strict packaging system such as `uv` or `cargo` , you should use apt to install dependency packages on Debian and Ubuntu whenever possible. When doing so, provide the apt install command that you want the user to run. Note that in ROS, when using uv, you should generally use `--system-site-packages` as the default, and install only those packages that cannot be installed via apt as Python packages within the virtual environment
