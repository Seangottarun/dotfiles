" My Vim Configuration
""""""""""""""
"  Global Vim "
""""""""""""""

set nocompatible              " be iMproved, required
filetype plugin on            " load plugin according to filetype
" let workmode=1                " only load work safe plugins
let workmode=0

""""""""""""""
"  Plugins "
""""""""""""""
"" Vim-plug Installation
if !has('win32')
    if empty(glob('~/.vim/autoload/plug.vim'))
        silent !curl -fLo ~/.vim/autoload/plug.vim --create-dirs
                    \ https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
        autocmd VimEnter * PlugInstall --sync | source $MYVIMRC
    endif
endif

"" Install Plugins
call plug#begin('~/.vim/plugged')

""" NerdTree file manager
Plug 'preservim/nerdtree'

""" Vim Airline
Plug 'vim-airline/vim-airline'
Plug 'vim-airline/vim-airline-themes'

""" Fugitive
Plug 'tpope/vim-fugitive'
" fugitive plugin for GitHub (:GBrowse for opening GitHub URLs)
Plug 'tpope/vim-rhubarb'

" vim-gitgutter and vim-signify seem to not work together
" gitgutter is nicer for git (stage hunks)
" signify supports all VCS
if workmode
    """ vim-signify
    if has('nvim') || has('patch-8.0.902')
        Plug 'mhinz/vim-signify'
    else
        Plug 'mhinz/vim-signify', { 'branch': 'legacy' }
    endif
else
    """ vim-gitgutter
    Plug 'airblade/vim-gitgutter'
endif

""" vim-pythonsense
Plug 'jeetsukumaran/vim-pythonsense'

""" fzf.vim
Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
Plug 'junegunn/fzf.vim'

""" commentary.vim
" gcc comment current line, gc to comment target of motion or visual selection
" gc is also a text object
Plug 'tpope/vim-commentary'

""" Verilog/SystemVerilog
Plug 'vhda/verilog_systemverilog.vim'

""" vim-better-whitespace.vim
" Run :StripWhitespace on entire file or visual selection
Plug 'ntpeters/vim-better-whitespace'

""" Tagbar class outline viewer for Vim using tags
Plug 'preservim/tagbar'

""" hl_matchit.vim
" Automatically highlights underlining matching words
Plug 'vimtaku/hl_matchit.vim'

""" surround.vim
" Change surroundings (parentheses, brackets, quotes, etc)
" cs: change surroundings
" ds: delete surroundings
" ys: you surround + vim motion or text object
" cs'" (change single to double quotes)
" yss" surround line in quotes
" S" for surrounding text in quotes in visual mode
Plug 'tpope/vim-surround'

""" vim-abolish
" Abbreviation, substitution, coercion (change case, ex. to camel_case)
" crc, crm, crs, cru to change word to camelCase, MixedCase, snake_case, and SNAKE_CASE
Plug 'tpope/vim-abolish'

""" vim-sneak
Plug 'justinmk/vim-sneak'

""" vim-markdown
" Plug 'godlygeek/tabular'
" Plug 'plasticboy/vim-markdown'

""" markdown-preview.nvim
" If you have nodejs and yarn
Plug 'iamcco/markdown-preview.nvim', { 'do': 'cd app && yarn install'  }

""" everforest
Plug 'sainnhe/everforest'

""" papercolor
" Plug 'NLKNguyen/papercolor-theme'

""" csv.vim
" Normal mode commands
" <Enter> dynamically folds all rows that don't match value in current column
" <Space> dynamically folds all rows that match value in current column
" :CSVTabularize to pretty print as table
" Use counts plus upper case H,J,K,L to move along columns/rows
Plug 'chrisbra/csv.vim'

""" vim-gutentags
" Autogenerate tags file (with incremental generation)
" :GutentagsUpdate and :GutentagsUpdate! for forcing an update of current tag file
" with current buffer and whole project, respectively
Plug 'ludovicchabant/vim-gutentags'

""" gutentags_plus
" Handles cscope databases
" <leader>cs 	Find symbol (reference) under cursor
" <leader>cg 	Find symbol definition under cursor
" <leader>cd 	Functions called by this function
" <leader>cc 	Functions calling this function
" <leader>ct 	Find text string under cursor
" <leader>ce 	Find egrep pattern under cursor
" <leader>cf 	Find file name under cursor
" <leader>ci 	Find files #including the file name under cursor
" <leader>ca 	Find places where current symbol is assigned
" <leader>cz 	Find current word in ctags database
" Plug 'skywind3000/gutentags_plus'

""" vim-dirdiff
" :DirDiff <dir1> <dir2>
" vim -c "DirDiff dir1 dir2"
Plug 'will133/vim-dirdiff'

""" ALE
Plug 'dense-analysis/ale'

if !workmode
    """ Vim-latex
    Plug 'vim-latex/vim-latex'

    """ Vimtex
    "Plug 'lervag/vimtex'
    """ UltiSnip Engine
    Plug 'SirVer/ultisnips'

    """ Gruvbox
    " Plug 'jordanhong/gruvbox-material'

    """ Ctrl-p (fuzzy search, uninstalled by default)
    " Plug 'ctrlpvim/ctrlp.vim'

    """ textobj-latex
    Plug 'kana/vim-textobj-user'
    Plug 'rbonvall/vim-textobj-latex'

    """ tex-conceal
    Plug 'KeitaNakamura/tex-conceal.vim', {'for': 'tex'}
endif

call plug#end()


"" Plug-in Settings
""" NerdTree
" Looks like this colon might be needed for NerdTreeToggle to work?
" Without it, <c-f> didn't seem to work.
nmap <c-f> :NERDTreeToggle<CR>
" Change CWD whenever tree root is changed
let NERDTreeChDirMode=2

""" Vim-latex
"let g:Tex_CompileRule_pdf = 'pdflatex -synctex=1 --interaction=batchmode $*'
let g:Tex_CompileRule_pdf = 'pdflatex -interaction=nonstopmode -file-line-error-style $*'
let g:Tex_DefaultTargetFormat = 'pdf'
if has('mac')
    let g:Tex_ViewRule_pdf = 'open -a Skim'
else
    let g:Tex_ViewRule_pdf = 'zathura'
endif
let g:tex_flavor = 'latex'
let g:Tex_Menus = 1
let g:Tex_SmartKeyQuote=0

""" Vimtex
"Mac uses Skim, Ubuntu uses Zathura
"let g:vimtex_view_method = 'skim'
" Required for vim to recognized latex files
"let g:tex_flavor = 'latex'
"let g:tex_conceal='abdmg'
" keep focus on vim after compiling
"let g:vimtex_view_automatic = 0

""" Ulti-snips
" Trigger configuration.
" Defaults:
" let g:UltiSnipsExpandTrigger="<tab>"
" let g:UltiSnipsJumpForwardTrigger="<C-j>"
" let g:UltiSnipsJumpBackwardTrigger="<C-k>"

let g:UltiSnipsListSnippets="<c-h>"
"let g:UltiSnipsEditSplit="vertical" "Split ultisnipedit vertically
" let g:UltiSnipsSnippetDirectories=[$HOME.'/.vim/UltiSnips']
" If needed, use UltiSnipsEdit to find the default location to put the snippets dir.
" You can also check currently loaded snippets with
" :echo " UltiSnips#SnippetsInCurrentScope()
"let g:UltiSnipsUsePythonVersion = 3

""" Everforest
if exists('+termguicolors')
    set termguicolors
endif
" May need to add to runtimepath
" set runtimepath+=~/.vim/plugged/everforest/colors
" Set contrast.
" This configuration option should be placed before `colorscheme everforest`.
" Available values: 'hard', 'medium'(default), 'soft'
let g:everforest_background = 'soft'
set background=dark
let g:everforest_enable_italic = 1
let g:everforest_disable_italic_comment = 1
colorscheme everforest
let g:airline_theme = 'everforest'

""" Papercolor
" colorscheme PaperColor
" let g:airline_theme='papercolor'

""" Gruvbox material
" if exists('+termguicolors')
"     set termguicolors
" endif
" if !workmode
"     let g:gruvbox_material_enable_bold = 1
" endif
" if has('mac')
"     set background=light
"     if !workmode
"         let g:gruvbox_material_disable_italic_comment = 1
"     endif
" else
"     if workmode
"         set background=light
"     else
"         set background=dark
"     endif
" endif
" if !workmode
"     colorscheme gruvbox-material
" endif

""" Vim Airline
" if workmode
"     let g:airline_theme='minimalist'
" else
"     let g:airline_theme='gruvbox_material'
" endif
if !exists('g:airline_symbols')
    let g:airline_symbols = {}
endif

" unicode symbols
let g:airline_left_sep = '»'
let g:airline_left_sep = '▶'
let g:airline_right_sep = '«'
let g:airline_right_sep = '◀'
let g:airline_symbols.crypt = '🔒'
let g:airline_symbols.linenr = '☰'
let g:airline_symbols.linenr = '␊'
let g:airline_symbols.linenr = '␤'
let g:airline_symbols.linenr = '¶'
let g:airline_symbols.maxlinenr = ''
let g:airline_symbols.maxlinenr = '㏑'
let g:airline_symbols.branch = '⎇'
let g:airline_symbols.paste = 'ρ'
let g:airline_symbols.paste = 'Þ'
let g:airline_symbols.paste = '∥'
let g:airline_symbols.spell = 'Ꞩ'
let g:airline_symbols.notexists = 'Ɇ'
let g:airline_symbols.whitespace = 'Ξ'

""" Ctrl-P
" let g:ctrlp_arg_map = 1
" let g:hardtime_ignore_buffer_patterns = [ "CustomPatt[ae]rn", "NERD.*" ]

""" ALE
noremap <F8> <ESC> :ALEFix <cr>
noremap! <F8> <ESC> :ALEFix <cr>
let g:ale_fixers = {
            \   '*': ['remove_trailing_lines', 'trim_whitespace'],
            \   'c': ['clang-format'],
            \   'cpp': ['clang-format'],
            \   'tex': ['latexindent']
            \}
let g:ale_c_clangformat_options = '-style="{IndentWidth: 4}"'

""" vim-gitgutter
" 100 ms update time so that signs change faster than default 4000ms
set updatetime=100

""" tex-conceal
set conceallevel=2
let g:tex_conceal='abdmg'
let g:tex_conceal_frac=1

""" tagbar
nmap <c-e> :TagbarToggle<CR>
" Sort tags by appearance instead of alphabetical order.
" Toggle sorting with `s`
let g:tagbar_sort = 0
" Note: Tagbar works best with universal ctags, so may have to set bin path.
" let g:tagbar_ctags_bin=""

""" vim-gutentags
" let g:gutentags_enabled = 0
" let g:gutentags_ctags_executable
" let g:gutentags_gtags_cscope_executable
" let g:gutentags_project_root=['']

""" gutentags_plus
" enable gtags module
" let g:gutentags_modules=['ctags', 'gtags_cscope']
" let g:gutentags_define_advanced_commands = 1

""" Verilog/SystemVerilog
let g:verilog_syntax_fold_lst = "all"

""" vim-signify
" Faster sign updates on CursorHold/CursorHoldI
set updatetime=100

nnoremap <leader>gd :SignifyDiff<cr>
nnoremap <leader>gp :SignifyHunkDiff<cr>
nnoremap <leader>gu :SignifyHunkUndo<cr>

" hunk jumping
nmap <leader>gj <plug>(signify-next-hunk)
nmap <leader>gk <plug>(signify-prev-hunk)

" hunk text object
omap ic <plug>(signify-motion-inner-pending)
xmap ic <plug>(signify-motion-inner-visual)
omap ac <plug>(signify-motion-outer-pending)
xmap ac <plug>(signify-motion-outer-visual)

""""""""""""""
"  General Settings  "
""""""""""""""
"" Status Bar
""" Show command
"Show what command is being typed
set showcmd

" Clipboard
set clipboard=unnamed

"" Mute bell
set belloff=all

"" Syntax
""" Indentation
"Set smart indent
set smartindent
set autoindent

""" Syntax Highlight
syntax on

""" Configure tab spaces
set tabstop=4
set shiftwidth=4
set expandtab

""" Writing
" Enable latex and markdown files
" autocmd FileType tex,markdown,text set spell
set spell spelllang=en_ca
set spell
inoremap <C-l> <c-g>u<Esc>[s1z=`]a<c-g>u

" Set word wrapping: prevent words from splitting off a line
set wrap
set textwidth=80

""" File Encoding
set encoding=utf-8
set fileencodings=utf-8

""" Backspace
if has('win32')
    set backspace=indent,eol,start
endif

"" Sessions
""" Update Session
nmap <F2> :wa<Bar>exe "mksession! " . v:this_session<CR>

"" File navigation
""" matchit (jump between keywords using %)
runtime macros/matchit.vim
" Alternative:
" packadd! matchit

""" Folding
"" Save folding state
nmap <F3> :mkview<CR>
nmap <F4> :loadview<CR>
nmap <F5> zfa(

""" Searching
" Set highlight search
set hls
" Set increment highlight matches
set incsearch
" Clears highlight of search when Enter.
" remap enter after search to clear highlight, then clears command.
nnoremap <silent> <cr> :noh<CR>

""" Wild menu
" Set wildmenu (shows completion list bar)
set wildmenu

""" Line number
"Toggle number at ctrl-l
"Hybrid: shows current abs number and relative
"see: :h number_relativenumber
"Show numbers by default
" set number! relativenumber!
" nmap <c-l> :set number! relativenumber!<CR>
" set number relativenumber
" Show relative numbers in normal mode and absolute numbers in insert mode
set number
autocmd InsertEnter * :set norelativenumber
autocmd InsertLeave * :set relativenumber

""" Tags
map <C-\> :bel vert winc ]<CR>
nnoremap <silent><Leader><C-]> <C-w><C-]><C-w>T

" HOW TO USE TAGS:
"Ctrl+] - go to definition
"Ctrl+T - Jump back from the definition.
"Ctrl+O - Jump to last place; Ctrl+I to reverse
"Ctrl+W Ctrl+] - Open the definition in a horizontal split
"Ctrl+\ - Open the definition in a vertical to the right (self-defined)
"<leader>Ctrl+] - Open the definition in a new tab (self-defined)

"Ctrl+W } : Opens a preview window with the location of the tag definition. The cursor does not change its position, so tag stack is not updated.
"Ctrl+W Z : Close preview window.
"Ctrl+W V : Split current window in two, keeping the cursor position.

" So, you can use <c-w>}if you want to quickly check the tag declaration,
" followed by <c-w>z to close it. But if you want to navigate, then you can
" simply use <c-w>v to create a split followed by the standard <c-] to navigate
" in the tags. When you're done with it, you can simply close the window with
" <c-w>c.

""" Mouse support
set mouse+=a

"" Backup
" Turn on backup to store in file.ext.bak
set backup
set backupext=.bak

"" Ctags shortcuts
"command! Maketag !ctags -R .
command! Maketag !ctags -R --c++-kinds=+p --fields=+iaS --extra=+q .
"command! Maketag !ctags --extra=+pf -R .
"command! Maketag !ctags -R . --c++-kinds=+pf --fields=+imaSft --extras=+q
"command! Maketag !ctags -R . --c++-kinds=+pf --fields=+imaSft --extra=+q
set tags=tags;/
set autochdir
set shortmess=a

"" Autocomplete
" by default, dashes ('-') aren't included in C-n C-p autocomplete
set iskeyword+=\-   "sets to recognize dashes

"" Python path
" Check for python dynamic support with :echo has("python3_dynamic").
" If using pyenv, do pyenv which python to find python home and dll.
" set pythonthreehome=C:\randomplace\python
" set pythonthreedll=C:\randomplace\python\python38.dll
" set pythonthreehome=C:\Users\i5_4670k\.pyenv\pyenv-win\versions\3.8.1
" set pythonthreedll=C:\Users\i5_4670k\.pyenv\pyenv-win\versions\3.8.1\python38.dll

"" Abbreviations
iab w/ with
iab w/o/ without
iab e/o/ each other

"" zj/zk jump to next closed fold
" Supports counts like [count]zj
nnoremap <silent> zj :call NextClosedFold('j')<cr>
nnoremap <silent> zk :call NextClosedFold('k')<cr>
" Supports counts like [count]<leader>zj
" nnoremap <silent> <leader>zj :call NextClosedFold('j')<cr>
" nnoremap <silent> <leader>zk :call NextClosedFold('k')<cr>

function! NextClosedFold(dir)
    let cmd = 'norm!z' . a:dir
    let view = winsaveview()
    let [l0, l, open] = [0, view.lnum, 1]
    while l != l0 && open
        exe cmd
        let [l0, l] = [l, line('.')]
        let open = foldclosed(l) < 0
    endwhile
    if open
        call winrestview(view)
    endif
endfunction

function! RepeatCmd(cmd) range abort
    let n = v:count < 1 ? 1 : v:count
    while n > 0
        exe a:cmd
        let n -= 1
    endwhile
endfunction

nnoremap <silent> zj :<c-u>call RepeatCmd('call NextClosedFold("j")')<cr>
nnoremap <silent> zk :<c-u>call RepeatCmd('call NextClosedFold("k")')<cr>
" nnoremap <silent> <leader>zj :<c-u>call RepeatCmd('call NextClosedFold("j")')<cr>
" nnoremap <silent> <leader>zk :<c-u>call RepeatCmd('call NextClosedFold("k")')<cr>

""""""""""""""""""""""""
"  Vimrc Organization  "
""""""""""""""""""""""""
"" Autofolding .vimrc
" see http://vimcasts.org/episodes/writing-a-custom-fold-expression/
""" defines a foldlevel for each line of code
function! VimFolds(lnum)
    let s:thisline = getline(a:lnum)
    if match(s:thisline, '^"" ') >= 0
        return '>2'
    endif
    if match(s:thisline, '^""" ') >= 0
        return '>3'
    endif
    let s:two_following_lines = 0
    if line(a:lnum) + 2 <= line('$')
        let s:line_1_after = getline(a:lnum+1)
        let s:line_2_after = getline(a:lnum+2)
        let s:two_following_lines = 1
    endif
    if !s:two_following_lines
        return '='
    endif
else
    if (match(s:thisline, '^"""""') >= 0) &&
                \ (match(s:line_1_after, '^"  ') >= 0) &&
                \ (match(s:line_2_after, '^""""') >= 0)
        return '>1'
    else
        return '='
    endif
endif
endfunction

""" defines a foldtext
function! VimFoldText()
    " handle special case of normal comment first
    let s:info = '('.string(v:foldend-v:foldstart).' l)'
    if v:foldlevel == 1
        let s:line = ' ◇ '.getline(v:foldstart+1)[3:-2]
    elseif v:foldlevel == 2
        let s:line = '   ●  '.getline(v:foldstart)[3:]
    elseif v:foldlevel == 3
        let s:line = '     ▪ '.getline(v:foldstart)[4:]
    endif
    if strwidth(s:line) > 80 - len(s:info) - 3
        return s:line[:79-len(s:info)-3+len(s:line)-strwidth(s:line)].'...'.s:info
    else
        return s:line.repeat(' ', 80 - strwidth(s:line) - len(s:info)).s:info
    endif
endfunction

""" set foldsettings automatically for vim files
augroup fold_vimrc
    autocmd!
    autocmd FileType vim
                \ setlocal foldmethod=expr |
                \ setlocal foldexpr=VimFolds(v:lnum) |
                \ setlocal foldtext=VimFoldText() |
    "              \ set foldcolumn=2 foldminlines=2
augroup END
