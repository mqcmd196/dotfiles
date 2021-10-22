(when window-system
  (tool-bar-mode -1)
)

(require 'all-the-icons)

(require 'doom-modeline)
(setq doom-modeline-project-detection 'projectile)
(setq doom-modeline-buffer-file-name-style 'truncate-except-project)
(setq doom-modeline-buffer-encoding nil)
(setq doom-modeline-major-mode-icon nil)
(setq doom-modeline-minor-mode-icon nil)
(setq doom-modeline-buffer-state-icon nil)
(setq doom-modeline-buffer-modification-icon nil)
(setq doom-modeline-unicode-fallback nil)
(setq doom-modeline-minor-modes nil)
(setq doom-modeline-vcs-max-length 10)
(setq doom-modeline-workspace-name nil)
(setq doom-modeline-persp-icon t)
(setq doom-modeline-irc-stylize 'identity)
(setq doom-modeline-height 20)
(doom-modeline-mode 1)
