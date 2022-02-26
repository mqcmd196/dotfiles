;;; $DOOMDIR/config.el -*- lexical-binding: t; -*-

;; Place your private configuration here! Remember, you do not need to run 'doom
;; sync' after modifying this file!


;; Some functionality uses this to identify you, e.g. GPG configuration, email
;; clients, file templates and snippets.
(setq user-full-name "Yoshiki Obinata"
      user-mail-address "mqcmd196@hotmail.co.jp")

;; Doom exposes five (optional) variables for controlling fonts in Doom. Here
;; are the three important ones:
;;
;; + `doom-font'
;; + `doom-variable-pitch-font'
;; + `doom-big-font' -- used for `doom-big-font-mode'; use this for
;;   presentations or streaming.
;;
;; They all accept either a font-spec, font string ("Input Mono-12"), or xlfd
;; font string. You generally only need these two:
;; (setq doom-font (font-spec :family "monospace" :size 12 :weight 'semi-light)
;;       doom-variable-pitch-font (font-spec :family "sans" :size 13))

;; There are two ways to load a theme. Both assume the theme is installed and
;; available. You can either set `doom-theme' or manually load a theme with the
;; `load-theme' function. This is the default:
(setq doom-theme 'doom-one)

;; If you use `org' and don't want your org files in the default location below,
;; change `org-directory'. It must be set before org loads!
(setq org-directory "~/org/")

;; This determines the style of line numbers in effect. If set to `nil', line
;; numbers are disabled. For relative line numbers, set this to `relative'.
(setq display-line-numbers-type t)


;; Here are some additional functions/macros that could help you configure Doom:
;;
;; - `load!' for loading external *.el files relative to this one
;; - `use-package!' for configuring packages
;; - `after!' for running code after a package has loaded
;; - `add-load-path!' for adding directories to the `load-path', relative to
;;   this file. Emacs searches the `load-path' when you load packages with
;;   `require' or `use-package'.
;; - `map!' for binding new keys
;;
;; To get information about any of these functions/macros, move the cursor over
;; the highlighted symbol at press 'K' (non-evil users must press 'C-c c k').
;; This will open documentation for it, including demos of how they are used.
;;
;; You can also try 'gd' (or 'C-c c d') to jump to their definition and see how
;; they are implemented.

;; Don't show initilal screen
(setq inhibit-startup-message t)
(setq initial-scratch-message nil)

;; Key bindings
(map! "\C-h" 'backward-delete-char)
(map! "\C-x\C-b" 'buffer-menu)
(global-set-key (kbd "C-M-g") 'google-this)
(global-set-key (kbd "M-%") 'anzu-query-replace)
(global-set-key (kbd "C-M-%") 'anzu-query-replace-regexp)
(global-set-key (kbd "C->") 'helm-ag-project-root)

;; History settings
(setq backup-directory-alist '((".*" . "~/.ehist")))
(setq auto-save-file-name-transforms   '((".*" "~/tmp/" t)))

 ;; Show emacs-shell on the same window
(add-to-list 'display-buffer-alist
             '("^\\*shell\\*$" . (display-buffer-same-window)))

;; use ccls as default
(after! ccls
  (setq ccls-initialization-options '(:index (:comments 2) :completion (:detailedLabel t)))
  (set-lsp-priority! 'ccls 2)) ; optional as ccls is the default in Doom

;; python version
(after! python
  (setq python-shell-interpreter "python"))

;; disable file watchers because sometimes it is so heavy
(after! lsp-mode
  (setq lsp-lens-enable nil)
  (setq! lsp-enable-file-watchers nil)
  (setq company-transformers nil company-lsp-async t company-lsp-cache-candidates nil))

;; flycheck after saved
(setq flycheck-check-syntax-automatically '(save mode-enable))

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