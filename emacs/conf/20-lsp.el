;; lsp-mode, language server protocol
(require 'lsp-mode)
(require 'flycheck)
(setq gc-cons-threshold 100000000)
(setq read-process-output-max (* 1024 1024))
(setq lsp-completion-provider :capf)
(setq lsp-idle-delay 0.500)
(setq lsp-log-io nil) ; if set to true can cause a performance hit
(setq lsp-diagnostic-provider :flycheck) ; use flycheck instead of flymake to improve the performance

;; lsp-ui
(require 'lsp-ui)
(setq lsp-ui-imenu-enable t)
(setq lsp-headerline-breadcrumb-enable nil)
