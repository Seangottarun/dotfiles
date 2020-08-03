set nocompatible
" Turn off Vi-compatibility mode and enable useful Vim functionality.

" Install vim-plug (comment this out on Windows)
if empty(glob('~/.vim/autoload/plug.vim'))
  silent !curl -fLo ~/.vim/autoload/plug.vim --create-dirs
    \ https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
  autocmd VimEnter * PlugInstall --sync | source $MYVIMRC
endif

" quicker window movement
nmap <C-j> <C-w>j
nmap <C-k> <C-w>k
nmap <C-h> <C-w>h
nmap <C-l> <C-w>l
nmap <C-t> :tabn<CR>


"---------------------
" Plugin configuration
"---------------------

call plug#begin('~/.vim/plugged')

" fuzzy finder fzf
Plug 'junegunn/fzf'
Plug 'junegunn/fzf.vim'
" If installed using Homebrew (comment out on Windows)
Plug '/usr/local/opt/fzf'
nmap <C-o> :Files<CR>
nmap <C-f> :Rg<CR>
nmap <C-b> :Buffers<CR>
nmap <C-s> :Snippets<CR>

" nerdtree
Plug 'preservim/nerdtree'
nmap <C-n> :NERDTreeToggle<CR>

" show hidden files by default
let NERDTreeShowHidden=1

" ignore specifc files
let NERDTreeIgnore=['\.pyc$', '\~$', '\.swp$']

Plug 'chriskempson/base16-vim'
let base16colorspace=256  " Access colors present in 256 colorspace

""" UltiSnip Engine
Plug 'SirVer/ultisnips'

" Trigger configuration. Do not use <tab> if you use
" https://github.com/Valloric/YouCompleteMe.
let g:UltiSnipsExpandTrigger="<tab>"
let g:UltiSnipsJumpForwardTrigger="<tab>"
let g:UltiSnipsJumpBackwardTrigger="<C-tab>"
let g:UltiSnipsSnippetDirectories=[$HOME.'/.vim/UltiSnips']
let g:UltiSnipsUsePythonVersion = 3

" ale
Plug 'dense-analysis/ale'
" Moves to next error/warning
nmap <C-e> <Plug>(ale_next_wrap)

" Toggle ALE quick list
noremap <C-i> :call QFixToggle()<CR>

function! QFixToggle()
  if exists("g:qfix_win")
    cclose
    unlet g:qfix_win
  else
    copen 10
    let g:qfix_win = bufnr("$")
  endif
endfunction


function ALE() abort
    return exists('*ALEGetStatusLine') ? ALEGetStatusLine() : ''
endfunction
let g:airline_section_error = '%{ALE()}'

let g:ale_lint_on_enter = 0
let g:ale_lint_on_save = 1

" Enable completion where available.
" This setting must be set before ALE is loaded.
"
" You should not turn this setting on if you wish to use ALE as a completion
" source for other completion plugins, like Deoplete.
let g:ale_completion_enabled = 1

"ALE provides an omni-completion function you can use for triggering completion
" manually with <C-x><C-o>.

set omnifunc=ale#completion#OmniFunc

let g:ale_sign_column_always = 1

let g:ale_fixers = {'tex': ['latexindent']}


" argwrap
"Plug 'FooSoft/vim-argwrap'
"nnoremap <C-w> :ArgWrap<CR>

" airline
Plug 'vim-airline/vim-airline'
Plug 'vim-airline/vim-airline-themes'
let g:airline_theme='base16'

" mundo
Plug 'simnalamburt/vim-mundo'
" Enable persistent undo so that undo history persists across vim sessions
set undofile
set undodir=~/.vim/undo
nnoremap <C-u> :MundoToggle<CR>

let g:mundo_width = 60
let g:mundo_preview_height = 40
let g:mundo_right = 1


" vim-tex
Plug 'lervag/vimtex'
"-----configure pdf application---------------------------
"Mac uses Skim, Ubuntu uses Zathura
let g:vimtex_view_method = 'skim'
" Required for vim to recognized latex files
let g:tex_flavor = 'latex'
let g:tex_conceal='abdmg'
" keep focus on vim after compiling
let g:vimtex_view_automatic = 0

call plug#end()

" Automatically install missing plugins on startup
autocmd VimEnter *
  \  if len(filter(values(g:plugs), '!isdirectory(v:val.dir)'))
  \|   PlugInstall --sync | q
  \| endif

" Turn on syntax highlighting.
syntax on

set showmatch " show matching braces when text indicator is over them

" Disable the default Vim startup message.
set shortmess+=I

" Show line numbers.
set number

" This enables relative line numbering mode. With both number and
" relativenumber enabled, the current line shows the true line number, while
" all other lines (above and below) are numbered relative to the current line.
" This is useful because you can tell, at a glance, what count is needed to
" jump up or down to a particular line, by {count}k to go up or {count}j to go
" down.
set relativenumber

" Always show the status line at the bottom, even if you only have one window open.
set laststatus=2

" The backspace key has slightly unintuitive behaviour by default. For example,
" by default, you can't backspace before the insertion point set with 'i'.
" This configuration makes backspace behave more reasonably, in that you can
" backspace over anything.
set backspace=indent,eol,start

" By default, Vim doesn't let you hide a buffer (i.e. have a buffer that isn't
" shown in any window) that has unsaved changes. This is to prevent you from "
" forgetting about unsaved changes and then quitting e.g. via `:qa!`. We find
" hidden buffers helpful enough to disable this protection. See `:help hidden`
" for more information on this.
set hidden

" This setting makes search case-insensitive when all characters in the string
" being searched are lowercase. However, the search becomes case-sensitive if
" it contains any capital letters. This makes searching more convenient.
set ignorecase
set smartcase

" Enable searching as you type, rather than waiting till you press enter.
set incsearch

" Unbind some useless/annoying default key bindings.
nmap Q <Nop> " 'Q' in normal mode enters Ex mode. You almost never want this.

" Disable audible bell because it's annoying.
set noerrorbells visualbell t_vb=

" Enable mouse support.
set mouse+=a

" Enable copying to clipboard (y and dd)
set clipboard+=unnamed 

" Toggle indentation when pasting text
set pastetoggle=<F3>

" Display invisible characters
set listchars=eol:¬,tab:>·,trail:~,extends:>,precedes:<,space:␣
" Windows version
" set listchars=eol:$,tab:>-,trail:~,extends:>,precedes:


" Toggle displayig invisible characters
noremap <F5> :set list!<CR>
inoremap <F5> <C-o>:set list!<CR>
cnoremap <F5> <C-c>:set list!<CR>

" Set vim to wrap text at 100 characters for markdown
autocmd bufreadpre *.md setlocal textwidth=100

" Set Indentation
set smartindent
set autoindent

" Configure tab spaces
set tabstop=4
set shiftwidth=4
set expandtab

