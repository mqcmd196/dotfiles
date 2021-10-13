;; company, completion tool
(require 'company)
(global-company-mode t)
(setq company-idle-delay 0.01)
(setq company-async-timeout 0.05)
(setq company-minimum-prefix-length 1)
(setq company-selection-wrap-around t)
(setq completion-ignore-case t)
(define-key company-active-map (kbd "M-n") 'company-select-next)
(define-key company-active-map (kbd "M-p") 'company-select-previous)
(define-key company-active-map (kbd "M-s") 'company-filter-candidates) ;; Maybe has another bind?
