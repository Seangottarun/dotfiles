" Avoid having cpp ftplugin source c.vim
if (&ft != 'c')
    finish
endif

setlocal foldmethod=syntax
set keywordprg=cppman
setlocal commentstring=//\ %s
