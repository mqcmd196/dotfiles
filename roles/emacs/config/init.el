;; packages
(require 'use-package)
(setq use-package-always-ensure nil) ;; Don't install packages automatically. Many ones are expected to be installed as apt packages

;; custom themes
(add-to-list 'custom-theme-load-path "~/dotfiles/non-sudoer/themes/obinata-deeper-blue")
(add-to-list 'load-path "~/dotfiles/non-sudoer/themes/obinata-deeper-blue")
(setq custom-theme-directory "~/dotfiles/non-sudoer/themes/obinata-deeper-blue")
(add-to-list 'load-path "~/dotfiles/non-sudoer/themes/emacs-one-themes")
(add-to-list 'custom-theme-load-path "~/dotfiles/non-sudoer/themes/emacs-one-themes")

(menu-bar-mode -1) ;; disable menu bar
(tool-bar-mode 0) ;; disable tool bar
(setq inhibit-startup-message t) ;; Don't show initial screen
(setq initial-scratch-message nil)
(show-paren-mode 1) ;; emphasize paren pair
(setq-default tab-width 4 indent-tabs-mode nil) ;; tab-width default
(setq-default c-basic-offset 4)
(electric-pair-mode 1) ;; Insert parenthesis/brackets by pair
(which-function-mode 1) ;; always show which function
(setq sgml-quick-keys 'close) ;; always show which function
(defalias 'yes-or-no-p 'y-or-n-p) ;; yes -> y, no -> n
(setq whitespace-space-regexp "\\(\u3000+\\)")
(setq warning-minimum-level :error) ;; suppress warning buffer
(setq visible-bell 1) ;; stop bell
(setq backup-directory-alist '((".*" . "~/tmp"))) ;; backup directory(
(setq auto-save-file-name-transforms   '((".*" "~/tmp/" t)))
(unless (window-system) ;; share clipboard when -nw. It requires xsel
  (setq x-select-enable-clipboard t))
(when (and (not (window-system)) ;; -nw mode
           (getenv "WSLENV")) ;; WSL environment
  (message "WSL: Enabling clipboard integration with clip.exe")
  (defun wsl-copy (text)
    (with-temp-buffer
      (insert text)
      (call-process-region (point-min) (point-max) "clip.exe" t 0)))
  (defun wsl-paste ()
    (shell-command-to-string "powershell.exe -NoProfile -Command 'Get-Clipboard' | tr -d '\\r'"))
  (setq interprogram-cut-function 'wsl-copy)
  (setq interprogram-paste-function 'wsl-paste)
  (setq select-active-regions nil))

;; ROS
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
(font-lock-add-keywords 'gdb-script-mode '(("\\<\\(bool\\|byte\\|int8\\|uint8\\|int16\\|uint16\\|int32\\|uint32\\|int64\\|uint64\\|float32\\|float64\\|string\\|time\\|duration\\)\\>" . font-lock-builtin-face)) 'set)

;; prog mode config
(add-hook 'prog-mode-hook ;; pretty symbols
          (lambda ()
            (setq prettify-symbols-alist '(("lambda" . ?λ))) ;; pretty symbols
            (prettify-symbols-mode 1)
            (hs-minor-mode t) ;; enable folding
            ))

(add-hook 'shell-mode-hook
          (lambda ()
            (company-mode -1)))

(use-package doom-themes
  :config
  (setq doom-themes-enable-bold t)
  (setq doom-themes-enable-italic t)
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
        (load-theme 'doom-one-light t)
      (load-theme 'doom-one t)))
  (when (display-graphic-p)
    (when (find-font (font-spec :name "Cascadia Code"))
      (set-face-attribute 'default nil :font "Cascadia Code-12"))
    (when (find-font (font-spec :family "BIZ UDGothic"))
      (set-fontset-font t 'japanese-jisx0208 (font-spec :family "BIZ UDGothic") nil 'prepend)
      (set-fontset-font t 'katakana-jisx0201 (font-spec :family "BIZ UDGothic") nil 'prepend))))

(use-package ligature ;; ligature. copied from https://github.com/mickeynp/ligature.el/wiki#cascadia--fira-code
  :config
  ;; Enable the "www" ligature in every possible major mode
  (ligature-set-ligatures 't '("www"))
  ;; Enable traditional ligature support in eww-mode, if the
  ;; `variable-pitch' face supports it
  (ligature-set-ligatures 'eww-mode '("ff" "fi" "ffi"))
  ;; Enable all Cascadia and Fira Code ligatures in programming modes
  (ligature-set-ligatures 'prog-mode
                          '(;; == === ==== => =| =>>=>=|=>==>> ==< =/=//=// =~
                            ;; =:= =!=
                            ("=" (rx (+ (or ">" "<" "|" "/" "~" ":" "!" "="))))
                            ;; ;; ;;;
                            (";" (rx (+ ";")))
                            ;; && &&&
                            ("&" (rx (+ "&")))
                            ;; !! !!! !. !: !!. != !== !~
                            ("!" (rx (+ (or "=" "!" "\." ":" "~"))))
                            ;; ?? ??? ?:  ?=  ?.
                            ("?" (rx (or ":" "=" "\." (+ "?"))))
                            ;; %% %%%
                            ("%" (rx (+ "%")))
                            ;; |> ||> |||> ||||> |] |} || ||| |-> ||-||
                            ;; |->>-||-<<-| |- |== ||=||
                            ;; |==>>==<<==<=>==//==/=!==:===>
                            ("|" (rx (+ (or ">" "<" "|" "/" ":" "!" "}" "\]"
                                            "-" "=" ))))
                            ;; \\ \\\ \/
                            ("\\" (rx (or "/" (+ "\\"))))
                            ;; ++ +++ ++++ +>
                            ("+" (rx (or ">" (+ "+"))))
                            ;; :: ::: :::: :> :< := :// ::=
                            (":" (rx (or ">" "<" "=" "//" ":=" (+ ":"))))
                            ;; // /// //// /\ /* /> /===:===!=//===>>==>==/
                            ("/" (rx (+ (or ">"  "<" "|" "/" "\\" "\*" ":" "!"
                                            "="))))
                            ;; .. ... .... .= .- .? ..= ..<
                            ("\." (rx (or "=" "-" "\?" "\.=" "\.<" (+ "\."))))
                            ;; -- --- ---- -~ -> ->> -| -|->-->>->--<<-|
                            ("-" (rx (+ (or ">" "<" "|" "~" "-"))))
                            ;; *> */ *)  ** *** ****
                            ("*" (rx (or ">" "/" ")" (+ "*"))))
                            ;; www wwww
                            ("w" (rx (+ "w")))
                            ;; <> <!-- <|> <: <~ <~> <~~ <+ <* <$ </  <+> <*>
                            ;; <$> </> <|  <||  <||| <|||| <- <-| <-<<-|-> <->>
                            ;; <<-> <= <=> <<==<<==>=|=>==/==//=!==:=>
                            ;; << <<< <<<<
                            ("<" (rx (+ (or "\+" "\*" "\$" "<" ">" ":" "~"  "!"
                                            "-"  "/" "|" "="))))
                            ;; >: >- >>- >--|-> >>-|-> >= >== >>== >=|=:=>>
                            ;; >> >>> >>>>
                            (">" (rx (+ (or ">" "<" "|" "/" ":" "=" "-"))))
                            ;; #: #= #! #( #? #[ #{ #_ #_( ## ### #####
                            ("#" (rx (or ":" "=" "!" "(" "\?" "\[" "{" "_(" "_"
                                         (+ "#"))))
                            ;; ~~ ~~~ ~=  ~-  ~@ ~> ~~>
                            ("~" (rx (or ">" "=" "-" "@" "~>" (+ "~"))))
                            ;; __ ___ ____ _|_ __|____|_
                            ("_" (rx (+ (or "_" "|"))))
                            ;; Fira code: 0xFF 0x12
                            ("0" (rx (and "x" (+ (in "A-F" "a-f" "0-9")))))
                            ;; Fira code:
                            "Fl"  "Tl"  "fi"  "fj"  "fl"  "ft"
                            ;; The few not covered by the regexps.
                            "{|"  "[|"  "]#"  "(*"  "}#"  "$>"  "^="))
  ;; Enables ligature checks globally in all buffers. You can also do it
  ;; per mode with `ligature-mode'.
  (global-ligature-mode t))

(use-package yasnippet
  :config
  (add-hook 'prog-mode-hook #'yas-minor-mode)
  (yas-global-mode 1))

(use-package company ;; completion
  :after yasnippet
  :init
  (add-hook 'after-init-hook 'global-company-mode)
  :config
  (setq company-minimum-prefix-length 2
	    company-idle-delay 0.01)
  (with-eval-after-load 'company
    (setq company-backends
	      '((company-capf :separate company-dabbrev-code company-yasnippet)
	        company-files
            company-yasnippet))))

(use-package eglot ;; LSP client
  :hook ((c-mode . eglot-ensure)
         (c++-mode . eglot-ensure)
         (python-mode . eglot-ensure)
         (js-mode . eglot-ensure)
         (typescript-mode . eglot-ensure))
  :init
  (setq gc-cons-threshold (* 10 1024 1024);; 100MB
	    read-process-output-max (* 2048 2048) ;; 4MB
	    eglot-events-buffer-config '(:size 0))
  :config
  (setq
   eglot-autoshutdown t
   eglot-stay-out-of '(company-backends)) ;; don't let eglot overwrite company-backends
  (let ((clangd-args '("clangd"
                       "-j=3"
                       "--background-index"
                       "--completion-style=detailed"
                       "--header-insertion=never"
                       "--header-insertion-decorators=0"
                       "--query-driver=/usr/bin/g++,/usr/bin/gcc,/usr/bin/c++")))
    (when (string= (getenv "ROS_VERSION") "1")
      (let* ((package-path (ignore-errors (string-trim-right (shell-command-to-string "roscd"))))
             (workspace-path (and package-path (expand-file-name ".." package-path)))
             (compile-db-path (and workspace-path (expand-file-name "compile_commands.json" workspace-path))))
        (when (and compile-db-path (file-exists-p compile-db-path))
          (push (format "--compile-commands-dir=%s" workspace-path) (cdr clangd-args))
          (message "EGLOT: ROS1 mode - Using compile_commands.json from %s" workspace-path))))
    (setq eglot-server-programs
          `((c-mode . ,clangd-args)
            (c++-mode . ,clangd-args)
            (python-mode . ("pylsp"))
            (js-mode . ("typescript-language-server" "--stdio"))
            (typescript-mode . ("typescript-language-server" "--stdio"))
            (js-ts-mode . ("typescript-language-server" "--stdio"))))))

(use-package anzu
  :config
  (global-anzu-mode 1)
  (global-set-key [remap query-replace] 'anzu-query-replace)
  (global-set-key [remap query-replace-regexp] 'anzu-query-replace-regexp))

(use-package magit)

(use-package diff-hl
  :config
  (global-diff-hl-mode)
  (add-hook 'dired-mode-hook 'diff-hl-dired-mode)
  (unless (window-system) (diff-hl-margin-mode)))

(use-package helm
  :init
  (setq helm-display-buffer-default-height 15
        helm-split-window-inside-p nil
        helm-split-window-default-side 'below
        helm-split-window-preferred-function #'split-window-below
        helm-ff-initial-sort-method 'newest)
  :bind
  (("M-x" . helm-M-x)
   ("C-x C-f" . helm-find-files)
   ("C-x b" . helm-mini)
   :map helm-map
   ("<tab>" . helm-execute-persistent-action) ; GUI
   ("C-i" . helm-execute-persistent-action) ; TTY
   ("C-z" . helm-select-action)
   :map helm-read-file-map ;; for find file
   ("<tab>" . helm-execute-persistent-action)
   ("C-i" . helm-execute-persistent-action)
   ("C-z" . helm-select-action))
  :config
  (add-to-list 'display-buffer-alist
               '("\\`\\*helm.*?\\*\\'"
                 (display-buffer-reuse-window display-buffer-in-side-window)
                 (side . bottom)
                 (slot . 0)
                 (window-height . 0.25)
                 (inhibit-same-window . t))) ;; show helm in buffer
  (helm-mode 1))

(use-package find-file-in-project :after helm) ;; Use helm with (helm-mode 1). Nothing to do here

(use-package helm-ag
  :after helm
  :bind (("C-c p" . helm-do-ag-project-root)))

(use-package hl-todo
  :config
  ;; Copied from https://github.com/doomemacs/doomemacs/blob/master/modules/ui/hl-todo/config.el
  (setq hl-todo-highlight-punctuation ":"
        hl-todo-keyword-faces
        '(("TODO" warning bold)
          ("FIXME" error bold)
          ("REVIEW" font-lock-keyword-face bold)
          ("HACK" font-lock-constant-face bold)
          ("DEPRECATED" font-lock-doc-face bold)
          ("NOTE" success bold)
          ("BUG" error bold)
          ("XXX" font-lock-constant-face bold)))
  :hook (after-init . global-hl-todo-mode))

(use-package powerline
  :config
  (powerline-default-theme))

(use-package format-all
  :commands format-all-mode
  :hook (prog-mode . format-all-mode)
  :config
  (setq-default format-all-formatters
                '(("C"     (astyle "--mode=c"))
                  ("Shell" (shfmt "-i" "4" "-ci")))))

;; Keybinds
(defun delete-word (arg)
  (interactive "p")
  (delete-region (point) (progn (forward-word arg) (point))))
(defun backward-delete-word (arg)
  (interactive "p")
  (delete-word (- arg)))
(global-set-key (read-kbd-macro "<M-DEL>") 'backward-delete-word) ;; not kill-ring with M-DEL
(global-set-key (read-kbd-macro "<C-backspace>") 'delete-word) ;; not kill-ring with M-d
(global-set-key "\C-h" 'delete-backward-char) ;; C-h to delete
(global-set-key (kbd "C-x C-b") 'ibuffer) ;; call ibuffer in current window
