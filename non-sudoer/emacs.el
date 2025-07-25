(setq initial-scratch-message 'nil)
(setq inhibit-startup-message 't)
(menu-bar-mode -1)
(tool-bar-mode 0)

;; loading themes
(add-to-list 'custom-theme-load-path "~/dotfiles/non-sudoer/themes/obinata-deeper-blue")
(add-to-list 'load-path "~/dotfiles/non-sudoer/themes/obinata-deeper-blue")
(setq custom-theme-directory "~/dotfiles/non-sudoer/themes/obinata-deeper-blue")
(add-to-list 'load-path "~/dotfiles/non-sudoer/themes/emacs-one-themes")
(add-to-list 'custom-theme-load-path "~/dotfiles/non-sudoer/themes/emacs-one-themes")

;; when light mode, use one-light. else use deeper-blue
(let* ((os-color-scheme
        (or (getenv "OS_COLOR_SCHEME")
            (cond
             ;; WSL: check Windows theme
             ((getenv "WSLENV")
              (if (string-match "1"
                                (shell-command-to-string
                                 "powershell.exe Get-ItemProperty -Path \"HKCU:\\\\SOFTWARE\\\\Microsoft\\\\Windows\\\\CurrentVersion\\\\Themes\\\\Personalize\" -Name AppsUseLightTheme | grep AppsUseLightTheme | awk '{ print $3 }' | grep 1"))
                  "light"
                "dark"))

             ;; GNOME: check GTK theme
             ((string-match "dark"
                            (shell-command-to-string
                             "gsettings get org.gnome.desktop.interface gtk-theme"))
              "dark")

             ;; MacOS: check system theme
             ((string-match "Dark"
                            (shell-command-to-string
                             "defaults read -g AppleInterfaceStyle 2>/dev/null"))
              "dark")

             ;; Default to light theme if none of the above matched
             (t "light")))))
  (if (string= os-color-scheme "light")
      (load-theme 'one-light t)
    ;; use obinata-deeper-blue
    (load-theme 'obinata-deeper-blue t)
    ;; black background
    (set-background-color "#2E3436")
    ;; black background in terminal
    (defun on-after-init ()
      (unless (display-graphic-p (selected-frame))
        (set-face-background 'default "unspecified-bg" (selected-frame))))
    (add-hook 'window-setup-hook 'on-after-init)))

;; Use Cascadia Code by default if exists
(cond
 ((find-font (font-spec :name "Cascadia Code"))
  (set-frame-font "Cascadia Code-12")))

;; backup directory
(setq backup-directory-alist '((".*" . "~/tmp")))
(setq auto-save-file-name-transforms   '((".*" "~/tmp/" t)))

(setq whitespace-space-regexp "\\(\u3000+\\)")

;; emphasize paren pair
(show-paren-mode 1)

;; tab-width default
(setq-default tab-width 4 indent-tabs-mode nil)
(setq-default c-basic-offset 4)

;; Insert parenthesis/brackets by pair
(electric-pair-mode 1)

;; always show which function
(which-function-mode 1)

;; auto add closing tag
(setq sgml-quick-keys 'close)

;; folding
(add-hook 'c++-mode-hook
          '(lambda ()
             (hs-minor-mode 1)))
(add-hook 'c-mode-hook
          '(lambda ()
             (hs-minor-mode 1)))
(add-hook 'scheme-mode-hook
          '(lambda ()
             (hs-minor-mode 1)))
(add-hook 'emacs-lisp-mode-hook
          '(lambda ()
             (hs-minor-mode 1)))
(add-hook 'lisp-mode-hook
          '(lambda ()
             (hs-minor-mode 1)))
(add-hook 'python-mode-hook
          '(lambda ()
             (hs-minor-mode 1)))
(add-hook 'ruby-mode-hook
          '(lambda ()
             (hs-minor-mode 1)))
(add-hook 'xml-mode-hook
          '(lambda ()
             (hs-minor-mode 1)))

;; yes -> y, no -> n
(defalias 'yes-or-no-p 'y-or-n-p)

;; yaml-mode, nxml in ros
(when (require 'yaml-mode nil t)
  (add-to-list 'auto-mode-alist '("\\.yml$" . yaml-mode)))
(add-to-list 'auto-mode-alist '("\.launch$" . nxml-mode))
(add-to-list 'auto-mode-alist '("\.test$" . nxml-mode))
(add-to-list 'auto-mode-alist '("manifest.xml" . nxml-mode))
(add-to-list 'auto-mode-alist '("\\.urdf" . xml-mode))
(add-to-list 'auto-mode-alist '("\\.xacro" . xml-mode))
(add-to-list 'auto-mode-alist '("\\.msg\\'" . gdb-script-mode))
(add-to-list 'auto-mode-alist '("\\.srv\\'" . gdb-script-mode))
(add-to-list 'auto-mode-alist '("\\.action\\'" . gdb-script-mode))
(font-lock-add-keywords 'gdb-script-mode                        '(("\\<\\(bool\\|byte\\|int8\\|uint8\\|int16\\|uint16\\|int32\\|uint32\\|int64\\|uint64\\|float32\\|float64\\|string\\|time\\|duration\\)\\>" . font-lock-builtin-face)) 'set)

;; Keybinds
;; C-h to delete
(global-set-key "\C-h" 'delete-backward-char)
;; call ibuffer in current window
(global-set-key (kbd "C-x C-b") 'ibuffer)

;; folding
(global-set-key (kbd "C-c C-f C-f") 'hs-toggle-hiding)
(global-set-key (kbd "C-c C-f C-u") 'hs-show-block)
(global-set-key (kbd "C-c C-f C-a C-f") 'hs-hide-all)
(global-set-key (kbd "C-c C-f C-a C-u") 'hs-show-all)
(global-set-key (kbd "C-c C-l") 'hs-hide-level)
