;; lsp-mode, language server protocol
(require 'lsp-mode)
(setq gc-cons-threshold 100000000)
(setq read-process-output-max (* 1024 1024))
(setq lsp-completion-provider :capf)
(setq lsp-idle-delay 0.500)

;; lsp-ui
(require 'lsp-ui)
(setq lsp-ui-imenu-enable t)
(setq lsp-headerline-breadcrumb-enable t)

;; company, completion tool
(require 'company)
(global-company-mode t)
(setq company-idle-delay 0.500)
(setq company-minimum-prefix-length 1)
(setq company-selection-wrap-around t)
(setq completion-ignore-case t)
(define-key company-active-map (kbd "M-n") 'company-select-next)
(define-key company-active-map (kbd "M-p") 'company-select-previous)
(define-key company-active-map (kbd "M-s") 'company-filter-candidates) ;; Maybe has another bind?

;; Python
(require 'lsp-python-ms)
(add-hook 'hack-local-variables-hook
          (lambda ()
            (when (derived-mode-p 'python-mode)
              (require 'lsp-python-ms)
              (lsp)))) ; or lsp-deferred
(setq lsp-python-ms-auto-install-server t)
(lsp-register-custom-settings
 `(("python.analysis.cachingLevel" lsp-python-ms-cache)
   ("python.analysis.errors" lsp-python-ms-errors)
   ("python.analysis.warnings" lsp-python-ms-warnings)
   ("python.analysis.information" lsp-python-ms-information)
   ("python.analysis.disabled" lsp-python-ms-disabled)
   ("python.autoComplete.extraPaths" lsp-python-ms-extra-paths)
   ("python.analysis.autoSearchPaths" ,(<= (length lsp-python-ms-extra-paths) 0) t)))
