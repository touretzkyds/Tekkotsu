TEKKOTSU_ROOT:=../..

VERSION_NUMBER_SRC:=$(TEKKOTSU_ROOT)/docs/doxygencfg
MODELS:=ERS7

all: $(addsuffix .pdf,$(addprefix VisionQuickReference-,$(MODELS)))
.PHONY: all clean

VisionQuickReference-%.pdf : VisionQuickReference.tex VersionNumber.tex vision.pdf
	@MODEL=`echo $* | sed 's/\([A-Z]\)\([0-9]\)/\1-\2/g'`; \
	echo "\newcommand{\model}{$$MODEL}" > ModelSpecific.tex; \
	echo "\newcommand{\condensedmodel}{$*}" >> ModelSpecific.tex; \
	echo "\newcommand{\release}{\input{VersionNumber}}" >> ModelSpecific.tex;
	pdflatex -interaction errorstopmode $< < /dev/null
	mv $(basename $<).pdf $@

VersionNumber.tex : $(VERSION_NUMBER_SRC)
	sed -n 's@PROJECT_NUMBER.*=[ 	]*\([0-9.]*\)@\1@p' $(VERSION_NUMBER_SRC) > $@

ifeq ($(findstring Darwin,$(shell uname)),)
vision.pdf: vision.dot
	dot -Tps2 $^ | ps2pdf - $@
else
vision.pdf: vision.dot
	dot -Tepdf $^ > $@
endif

clean:
	rm -f *~ *.aux *.log *.out vision.pdf


