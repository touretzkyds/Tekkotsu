TEKKOTSU_ROOT:=../..

EVENT_GENERATOR_SRC:=$(TEKKOTSU_ROOT)/Events/EventBase.h
VERSION_NUMBER_SRC:=$(TEKKOTSU_ROOT)/docs/doxygencfg
MODELS:=ERS7 ERS210 ERS220

all: $(addsuffix .pdf,$(addprefix TekkotsuQuickReference-,$(MODELS)))
.PHONY: all clean

TekkotsuQuickReference-%.pdf : TekkotsuQuickReference.tex EventGenerators.tex VersionNumber.tex
	@MODEL=`echo $* | sed 's/\([A-Z]\)\([0-9]\)/\1-\2/g'`; \
	echo "\\\\newcommand{\\\\model}{$$MODEL}" > ModelSpecific.tex; \
	echo "\\\\newcommand{\\\\condensedmodel}{$*}" >> ModelSpecific.tex; \
	echo "\\\\newcommand{\\\\release}{\\\\input{VersionNumber}}" >> ModelSpecific.tex;
	pdflatex -interaction errorstopmode $< < /dev/null
	mv $(basename $<).pdf $@

# The first part of this sed script extracts the EventGeneratorID_t enumeration
# and formats it as a latex list.  That output is then put through a second sed
# script which sanitizes special characters such as '_'.
EventGenerators.tex : $(EVENT_GENERATOR_SRC)
	sed -n '/enum EventGeneratorID_t/,/numEGIDs/s@	*\([a-zA-Z]*EGID\)\(=0\)\{0,1\},.*//!< *\(.*\)@{\\tt \1}, @p' $< | sed 's/_/\\_/g' > $@

VersionNumber.tex : $(VERSION_NUMBER_SRC)
	sed -n 's@PROJECT_NUMBER.*=[ 	]*\([0-9.]*\)@\1@p' $(VERSION_NUMBER_SRC) > $@

clean:
	rm -f *~ *.aux *.log *.out


