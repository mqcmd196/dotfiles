(when window-system
  (scroll-bar-mode -1) ;  hide scroll bar
  (menu-bar-mode -1) ; メニューバーを消す
  (tool-bar-mode -1) ; ツールバーを消す
)

(require 'doom-modeline)
(setq doom-modeline-project-detection 'projectile)
(setq doom-modeline-buffer-file-name-style 'truncate-except-project)
(setq doom-modeline-buffer-encoding nil)
(setq doom-modeline-major-mode-icon nil)
(setq doom-modeline-minor-mode-icon nil)
(setq doom-modeline-buffer-state-icon nil)
(setq doom-modeline-buffer-modification-icon t)
(setq doom-modeline-unicode-fallback nil)
(setq doom-modeline-minor-modes nil)
(setq doom-modeline-vcs-max-length 12)
(setq doom-modeline-workspace-name nil)
(setq doom-modeline-persp-icon t)
(setq doom-modeline-irc-stylize 'identity)
(setq doom-modeline-height 10)
(doom-modeline-mode 1)

(require 'doom-themes)
(load-theme 'doom-badger t)
