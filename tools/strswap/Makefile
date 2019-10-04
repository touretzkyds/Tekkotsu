.PHONY: all clean test

NAME:=strswap

all: $(NAME)

$(NAME): $(NAME).cc
	$(CXX) -o $@ -Wall -O2 $^
	ln -fs ../$@/$@ ../sbin

clean:
	rm -f $(NAME)
	rm -f ../sbin/$(NAME)

test:

