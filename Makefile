BASE:=book
BOOKNAME:=`ls -1 $(BASE) | head -n 1`
SRC:=book/$(BOOKNAME)

all:
	cat README.md

include resources/makefiles/setup.Makefile
