;; emacs settings

(global-set-key "\C-h" 'backward-delete-char)
(global-set-key "\M-g" 'goto-line)
(global-unset-key "\C-o" )

;; use fetch1075's tab settings
(setq-default indent-tabs-mode nil)
(setq-default tab-width 2
			  c-basic-offiset 2
			  js-indent-level 2)
(add-hook 'python-mode-hook '(lambda ()
							   (setq python-indent-offset 4)))
(add-hook 'c-mode-common-hook '(lambda ()
								 (c-set-style "linux")
								 (setq c-basic-offset tab-width)))
(global-linum-mode t)

;; yaml mode
(require 'yaml-mode)
(add-to-list 'auto-mode-alist '("\\.yml\\'" . yaml-mode))
(require 'yaml-mode)
(add-to-list 'auto-mode-alist '("\\.yaml\\'" . yaml-mode))

;; save history at ~/
(setq backup-directory-alist '((".*" . "~/.ehist")))
(setq auto-save-file-name-transforms   '((".*" "~/tmp/" t)))

;; Don't show initilal screen
(setq inhibit-startup-message t)
(setq initial-scratch-message nil)

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
