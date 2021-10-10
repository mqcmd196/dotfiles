;; save history at HOME
(setq backup-directory-alist '((".*" . "~/.ehist")))
(setq auto-save-file-name-transforms   '((".*" "~/tmp/" t)))

;; Don't show initilal screen
(setq inhibit-startup-message t)
(setq initial-scratch-message nil)

(add-to-list 'display-buffer-alist
             '("^\\*shell\\*$" . (display-buffer-same-window))) ;; Show emacs-shell on the same window

(electric-pair-mode 1) ;; automatically add pair

;; save current cursol
(require 'saveplace)
(setq-default save-place t)

;; google
(require 'google-this)

(put 'upcase-region 'disabled nil)
