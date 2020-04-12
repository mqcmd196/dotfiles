"ファイルを上書きする前にバックアップを作ることを無効化
set nowritebackup
set nobackup
"vimの矩形選択で文字がなくても右に進める
set virtualedit=block
"挿入モードでバックスペースで削除できるようにする
set backspace=indent,eol,start
"全角文字専用の設定
set ambiwidth=double
"wildmenuオプションを有効
set wildmenu

"##表示設定##
"エラーメッセージの表示時にビーぷを鳴らさない
set noerrorbells
" 行番号を追加
set number
" 行が折り返し表示されていた場合、行単位ではなく表示行単位でカーソルを移動する
nnoremap j gj
nnoremap k gk
nnoremap <down> gj
nnoremap <up> gk
" カーソルの左右移動で行末から次の行の行頭への移動が可能になる
set whichwrap=b,s,h,l,<,>,[,],~
" カーソルラインハイライト
set cursorline
" 編集中のファイル名を表示
set title
"括弧入力時に対応する括弧を表示
set showmatch matchtime=1
"インデント方法の変更
set cinoptions+=0
"メッセージの表示欄を２行確保
set cmdheight=2
"ステータス行を常に表示
set laststatus=2
"ウィンドウの右下にまだ実行していない入力中のコマンドを表示
set showcmd
"省略されずに表示
set display=lastline
"コードの色分け
syntax on
"インデントをスペース4つ分に設定
set tabstop=2
"タブ入力を複数の空白入力に置き換える
set expandtab
" 連続した空白に対してタブキーやバックスペースキーでカーソルが動く幅
set softtabstop=4
"オートインデント
set smartindent
set autoindent
set shiftwidth=2
set wildmenu " コマンドモードの補完
set history=5000 " 保存するコマンド履歴の数
 
"##検索設定##
"大文字/小文字の区別なく検索
set ignorecase
"検索文字列に大文字が含まれている場合は区別して検索する
set smartcase
"検索時最後まで行ったら最初に戻る
set wrapscan
"インクリメンタル検索（検索ワードの最初の文字を入力した時点で検索が開始）
set incsearch
"検索結果をハイライト表示
set hlsearch

""""""""""""""""""""""""""""""
" プラグインのセットアップ
""""""""""""""""""""""""""""""
call plug#begin('~/.vim/plugged')

" ファイルオープンを便利に
"Plug 'Shougo/unite.vim'
" Unite.vimで最近使ったファイルを表示できるようにする
"Plug 'Shougo/neomru.vim'
" カラースキームmolokai
"Plug 'tomasr/molokai'
"カラースキームhybrid
Plug 'w0ng/vim-hybrid'
" ステータスラインの表示内容強化
"Plug 'itchyny/lightline.vim'
Plug 'vim-airline/vim-airline'
Plug 'vim-airline/vim-airline-themes'
Plug 'ryanoasis/vim-devicons'
" ファイルをtree表示してくれる
Plug 'scrooloose/nerdtree'
" 末尾の全角と半角の空白文字を赤くハイライト
Plug 'bronson/vim-trailing-whitespace'
"Plug 'neoclide/coc.nvim', {'do': { -> coc#util#install()}}
"ウィンドウサイズの調整"
Plug 'simeji/winresizer'
"end自動入力"
Plug 'tpope/vim-endwise'
"auto-pairs"
Plug 'jiangmiao/auto-pairs'
"ruby on rails用のプラグイン"
"Plug 'tpope/vim-rails'
Plug 'neoclide/coc.nvim', {'branch': 'release'}
Plug 'w0rp/ale'
"vim上でgitを使う
Plug 'tpop/vim-fugitive'

call plug#end()
""""""""""""""""""""""""""""""
"----------------------------------------------------------
" molokaiの設定
"----------------------------------------------------------

"colorscheme molokai " カラースキームにmolokaiを設定する

set t_Co=256 " iTerm2など既に256色環境なら無くても良い
syntax enable " 構文に色を付ける

"----------------------------------------------------------
"hybridの設定
"----------------------------------------------------------

"let g:hybrid_custom_term_colors = 1
"let g:hybrid_reduced_contrast = 1 " Remove this line if using the default palette.
set background=dark
colorscheme hybrid

"----------------------------------------------------------
" ステータスラインの設定
"----------------------------------------------------------
"set laststatus=2 " ステータスラインを常に表示
"set showmode " 現在のモードを表示
"set showcmd " 打ったコマンドをステータスラインの下に表示
"set ruler " ステータスラインの右側にカーソルの現在位置を表示する
let g:airline_theme = 'wombat'
set laststatus=2
let g:airline#extensions#branch#enabled = 1
let g:airline#extensions#tabline#enabled = 1
let g:airline#extensions#wordcount#enabled = 0
let g:airline#extensions#default#layout = [['a', 'b', 'c'], ['x', 'y', 'z']]
let g:airline_section_c = '%t'
let g:airline_section_x = '%{&filetype}'
let g:airline_section_z = '%3l:%2v %{airline#extensions#ale#get_warning()} %{airline#extensions#ale#get_error()}'
let g:airline#extensions#ale#error_symbol = ' '
let g:airline#extensions#ale#warning_symbol = ' '
let g:airline#extensions#default#section_truncate_width = {}
let g:airline#extensions#whitespace#enabled = 1

nnoremap <silent><C-e> :NERDTreeToggle<CR>
"autocmd bufenter * if (winnr("$") == 1 && exists("b:NERDTree") && b:NERDTree.isTabTree()) | q | endif
autocmd VimEnter * if argc() == 0 && !exists("s:std_in") | NERDTree | endif

let g:winresizer_start_key = '<C-T>'
let g:winresizer_vert_resize = 1
let g:winresizer_horiz_resize = 1

"C-l,C-hでタブ間の移動"
map <C-l> gt
map <C-h> gT

"-----------------------------------------------------------
"coc.nvimの設定
"-----------------------------------------------------------
let g:coc_global_extensions = ['coc-solargraph']

inoremap <silent><expr> <TAB>
      \ pumvisible() ? "\<C-n>" :
      \ <SID>check_back_space() ? "\<TAB>" :
      \ coc#refresh()
inoremap <expr><S-TAB> pumvisible() ? "\<C-p>" : "\<C-h>"

function! s:check_back_space() abort
  let col = col('.') - 1
  return !col || getline('.')[col - 1]  =~# '\s'
endfunction

"------------------------------------------------------------
"aleの設定
"------------------------------------------------------------
let g:ale_fixers = {
      \ 'ruby': ['rubocop'],
      \ }
" 保存時のみ実行する
let g:ale_lint_on_text_changed = 0
" 表示に関する設定
let g:ale_sign_error = ''
let g:ale_sign_warning = ''
let g:airline#extensions#ale#open_lnum_symbol = '('
let g:airline#extensions#ale#close_lnum_symbol = ')'
let g:ale_echo_msg_format = '[%linter%]%code: %%s'
highlight link ALEErrorSign Tag
highlight link ALEWarningSign StorageClass
" Ctrl + kで次の指摘へ、Ctrl + jで前の指摘へ移動
nmap <silent> <C-k> <Plug>(ale_previous_wrap)
nmap <silent> <C-j> <Plug>(ale_next_wrap)
