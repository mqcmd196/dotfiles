;; emacs settings
;; Added by Package.el.  This must come before configurations of
;; installed packages.  Don't delete this line.  If you don't want it,
;; just comment it out by adding a semicolon to the start of the line.
;; You may delete these explanatory comments.
(require 'package)
(add-to-list 'package-archives '("melpa-stable" . "https://stable.melpa.org/packages/") t)
(package-initialize)

(add-to-list 'load-path "/usr/local/share/emacs/site-lisp") ;; use shared packages

(global-set-key "\C-h" 'backward-delete-char)
(global-set-key "\M-g" 'goto-line)
(global-unset-key "\C-o" )
(global-set-key "\C-x\C-b" 'buffer-menu)
(add-to-list 'default-frame-alist
             '(font . "DejaVu Sans Mono-9"))
(add-to-list 'display-buffer-alist
             '("^\\*shell\\*$" . (display-buffer-same-window))) ;; Show emacs-shell on the same window

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
 '(package-selected-packages
   (quote
    (use-package company lsp-ui lsp-mode gnu-elpa-keyring-update))))
(custom-set-faces
 ;; custom-set-faces was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 '(default ((t (:inherit nil :stipple nil :background "black" :foreground "white" :inverse-video nil :box nil :strike-through nil :overline nil :underline nil :slant normal :weight normal :height 113 :width normal :foundry "unknown" :family "DejaVu Sans Mono"))))
 '(font-lock-function-name-face ((t (:foreground "cyan"))))
 '(minibuffer-prompt ((t (:foreground "color-33"))))
 '(term-color-blue ((t (:background "deep sky blue" :foreground "deep sky blue")))))

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
(put 'upcase-region 'disabled nil)

;; lsp
(add-to-list 'load-path "~/.emacs.d/conf")
(load "lsp")
