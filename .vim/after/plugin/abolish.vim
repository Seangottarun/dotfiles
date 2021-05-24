if !exists(":Abolish")
    finish
endif

" :Abolish {despa,sepe}rat{e,es,ed,ing,ely,ion,ions,or}  {despe,sepa}rat{}
" For some reason forward slashes don't work in Abolish and regular vim ab needs
" to with a slash if you put in a slash.
" iab w/ with
" iab w/o/ without
" iab e/o/ each other
" iab not working for now unfortunately
Abolish req{,s,d} require{,s,d}
Abolish b{ec,c} because
Abolish nec necessarily
Abolish elem{,s} element{,s}
Abolish succ{,l,ly} success{,ful,fully}
Abolish {comp,cmop,cmpo}{,s,u} {comp,comp,comp}{uter,uters,uting}
Abolish sci{,s,t} scien{ce,ces,tific}
Abolish geom geometric
Abolish perf performance
Abolish crit critical
Abolish {are,ca,could,did,do,has,have,is,should,wo}nt {are,ca,could,did,do,has,have,is,should,wo}n't
Abolish Im I'm
Abolish rep{,s} represent{,s}
Abolish res{,s} resource{,s}
Abolish lidsa Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.
