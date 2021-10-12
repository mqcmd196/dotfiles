;; Global set key
(global-set-key "\C-h" 'backward-delete-char)
(global-set-key "\M-g" 'goto-line)
(global-unset-key "\C-o" )
(global-set-key "\C-x\C-b" 'buffer-menu)

;; anzu
(global-set-key (kbd "M-%")   'anzu-query-replace)
(global-set-key (kbd "C-M-%") 'anzu-query-replace-regexp)

;; counsel
(global-set-key (kbd "M-x")   'counsel-M-x) ;; comment out to use M-x not installed pc
(global-set-key (kbd "M-y")   'counsel-yank-pop)
(global-set-key (kbd "C-.")  'counsel-projectile-ag)

;; helm
(global-set-key (kbd "C->")  'helm-ag-project-root)

;; google
(global-set-key (kbd "C-M-g") 'google-this)
