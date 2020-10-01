;; emacs settings

(global-set-key "\C-h" 'backward-delete-char)
(global-set-key "\M-g" 'goto-line)
(global-unset-key "\C-o" )

(setq rosdistro (getenv "ROS_DISTRO"))
(add-to-list 'load-path (format "/opt/ros/%s/share/emacs/site-lisp" (or rosdistro "melodic")))
(require 'rosemacs)
(invoke-rosemacs)
(global-set-key "\C-x\C-r" ros-keymap)

