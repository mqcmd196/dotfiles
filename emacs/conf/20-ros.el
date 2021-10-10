;; for ROS
(when (string= (getenv "ROS_DISTRO") "melodic")
  (setq rosdistro (getenv "ROS_DISTRO"))
  (add-to-list 'load-path (format "/opt/ros/%s/share/emacs/site-lisp" (or rosdistro "melodic")))
  (require 'rosemacs)
  (invoke-rosemacs)
  (global-set-key "\C-x\C-r" ros-keymap)

  (add-to-list 'load-path "/opt/ros/melodic/share/euslime")
  (require 'euslime-config)
  (setq inferior-euslisp-program "roseus")
  (slime-setup '(slime-fancy slime-banner slime-repl-ansi-color))
)
