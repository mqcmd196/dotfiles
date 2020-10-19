;; emacs settings

(global-set-key "\C-h" 'backward-delete-char)
(global-set-key "\M-g" 'goto-line)
(global-unset-key "\C-o" )

(setq default-tab-width 4)
(global-linum-mode t)
(setq indent-tabs-mode nil)

(setq rosdistro (getenv "ROS_DISTRO"))
(add-to-list 'load-path (format "/opt/ros/%s/share/emacs/site-lisp" (or rosdistro "melodic")))
(require 'rosemacs)
(invoke-rosemacs)
(global-set-key "\C-x\C-r" ros-keymap)

(add-to-list 'load-path "/opt/ros/melodic/share/euslime")
(require 'euslime-config)
(setq inferior-euslisp-program "roseus")
(slime-setup '(slime-fancy slime-banner slime-repl-ansi-color))

(electric-pair-mode 1)

(custom-set-variables
 ;; custom-set-variables was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 )
(custom-set-faces
 ;; custom-set-faces was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 '(font-lock-function-name-face ((t (:foreground "cyan"))))
 '(minibuffer-prompt ((t (:foreground "color-33")))))
