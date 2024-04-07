;;; $DOOMDIR/config.el -*- lexical-binding: t; -*-

;; Place your private configuration here! Remember, you do not need to run 'doom
;; sync' after modifying this file!


;; Some functionality uses this to identify you, e.g. GPG configuration, email
;; clients, file templates and snippets.
(setq user-full-name "Yoshiki Obinata"
      user-mail-address "mqcmd196@hotmail.co.jp")

(setq auth-sources '("~/.netrc.gpg"))

(let* ((os-color-scheme (getenv "OS_COLOR_SCHEME")))
  (if (string= os-color-scheme "light")
      (setq doom-theme 'doom-one-light)
    (setq doom-theme 'doom-one)))

;; font settings
;; + `doom-big-font' -- used for `doom-big-font-mode'; use this for
;;   presentations or streaming.
(setq doom-font (font-spec :family "Cascadia Code" :size 15 :weight 'normal)
      doom-big-font (font-spec :family "Cascadia Code" :size 20))
;; copilot suggestions
;; (setq doom-font (font-spec :family "Cascadia Code" :size 14)
;;       doom-variable-pitch-font (font-spec :family "Cascadia Code" :size 14)
;;       doom-big-font (font-spec :family "Cascadia Code" :size 20))

;; If you use `org' and don't want your org files in the default location below,
;; change `org-directory'. It must be set before org loads!
(setq org-directory "~/org/")

;; This determines the style of line numbers in effect. If set to `nil', line
;; numbers are disabled. For relative line numbers, set this to `relative'.
(setq display-line-numbers-type nil)


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

;; modeline
(after! doom-modeline
  (setq doom-modeline-buffer-file-name-style 'truncate-except-project)
  (setq doom-modeline-vcs-max-length 12)
  (setq doom-modeline-github t)
  (setq doom-modeline-github-interval (* 30 60))
  (setq doom-modeline-checker-simple-format t))

;; Key bindings
(map! "\C-h" 'backward-delete-char)
(global-set-key (kbd "C-M-g") 'google-this)
(global-set-key (kbd "M-%") 'anzu-query-replace)
(global-set-key (kbd "C-M-%") 'anzu-query-replace-regexp)
(global-set-key (kbd "M-,") '+default/search-project)
(global-set-key (kbd "M-.") '+default/search-cwd)
(global-set-key (kbd "C-c d") '+lookup/definition)

(defun delete-word (arg)
  (interactive "p")
  (delete-region (point) (progn (forward-word arg) (point))))
(defun backward-delete-word (arg)
  (interactive "p")
  (delete-word (- arg)))
(global-set-key (read-kbd-macro "<M-DEL>") 'backward-delete-word) ;; not kill-ring with M-DEL
(global-set-key [(meta d)] 'delete-word) ;; not kill-ring with M-d


;; History settings
(setq backup-directory-alist '((".*" . "~/.ehist")))
(setq auto-save-file-name-transforms   '((".*" "~/tmp/" t)))

;; Show emacs-shell on the same window
(add-to-list 'display-buffer-alist
             '("^\\*shell\\*$" . (display-buffer-same-window)))


;; LSP
(setq gc-cons-threshold 100000000) ;; 1MB
(setq read-process-output-max (* 1024 1024))
;; disable file watchers because sometimes it is so heavy
(after! lsp-mode
  (setq! lsp-lens-enable nil)
  (setq! lsp-enable-file-watchers nil)
  (setq! lsp-log-io nil)
  (setq! lsp-restart 'auto-restart)
  ;; make lsp-documentation popup smaller
  (set-popup-rules!
    '(("^\\*lsp-documentation\\*$"
       :size 0.2
       :side bottom
       :select t
       :quit t
       :ttl nil))))
;; make lsp-signature-render-documentation popup smaller

;; company
(after! company
  (setq company-dabbrev-downcase 0)
  (setq company-idle-delay 0)
  (define-key! company-active-map [tab] 'company-complete-selection)
  (define-key! company-active-map (kbd "TAB") 'company-complete-selection)
  (define-key! company-active-map (kbd "M-n") 'company-select-next-or-abort)
  (define-key! company-active-map (kbd "M-p") 'company-select-previous-or-abort)
  (setq company-global-modes '(not latex-mode)))

;; make flycheck popup smaller
(after! flycheck
  (set-popup-rules!
    '(("^\\*Flycheck errors\\*$"
       :size 0.07
       :side bottom
       :select t
       :quit t
       :ttl nil))))

;; word wrapping
(+global-word-wrap-mode +1)

;; use clangd as default
(setq lsp-clients-clangd-args '("-j=3"
                                "--background-index"
                                "--clang-tidy"
                                "--completion-style=detailed"
                                "--header-insertion=never"
                                "--header-insertion-decorators=0"
                                "--query-driver=/usr/bin/g++,/usr/bin/gcc,/usr/bin/c++"))
(after! lsp-clangd (set-lsp-priority! 'clangd 2))
;; clangd path
(if (executable-find "clangd")
    (setq lsp-clients-clangd-executable "clangd")
  (setq lsp-clients-clangd-executable "clangd-18"))

;; C/C++ paren settings
(after! cc-mode
  (add-hook 'c++-mode-hook #'(lambda ()
                               ;; disable auto-insertion of angle brackets)
                               (sp-local-pair 'c++-mode "<" nil :actions nil)
                               (c-set-offset 'innamespace 0)
                               (setq c-basic-offset 4))))

;; python version
(after! python
  (setq python-shell-interpreter "python"))

;; conda
(setq conda-env-home-directory (expand-file-name "~/miniconda3/"))

;; golang
(add-to-list 'exec-path (expand-file-name "~/go/bin"))

;; dart
(with-eval-after-load 'projectile
  (add-to-list 'projectile-project-root-files-bottom-up "pubspec.yaml")
  (add-to-list 'projectile-project-root-files-bottom-up "BUILD"))

;; github copilot
;; it requires new node to be installed, like sudo snap install node --classic
(use-package! copilot
  :hook (prog-mode . copilot-mode)
  :bind (:map copilot-completion-map
              ("C-e" . 'copilot-accept-completion)
              ("M-f" . 'copilot-accept-completion-by-word)))

;; dap mode
(after! dap-mode
  (setq dap-python-debugger 'debugpy))

;; for ROS
;; NOTE rosemacs doesn't support emacs28.1 now
(add-to-list 'auto-mode-alist '("\.launch$" . nxml-mode))
(add-to-list 'auto-mode-alist '("\.test$" . nxml-mode))
(add-to-list 'auto-mode-alist '("manifest.xml" . nxml-mode))
(add-to-list 'auto-mode-alist '("\\.urdf" . xml-mode))
(add-to-list 'auto-mode-alist '("\\.xacro" . xml-mode))
(add-to-list 'auto-mode-alist '("\\.msg\\'" . gdb-script-mode))
(add-to-list 'auto-mode-alist '("\\.srv\\'" . gdb-script-mode))
(add-to-list 'auto-mode-alist '("\\.action\\'" . gdb-script-mode))
(font-lock-add-keywords 'gdb-script-mode
                        '(("\\<\\(bool\\|byte\\|int8\\|uint8\\|int16\\|uint16\\|int32\\|uint32\\|int64\\|uint64\\|float32\\|float64\\|string\\|time\\|duration\\)\\>" . font-lock-builtin-face)) 'set)
