all:
	cat README.md

clean:
	rm -rf duckuments-dist out

build:
	docker run -it  -v $(CURDIR):/pwd:delegated --workdir /pwd  -e COMPMAKE_COMMAND=rparmake duckietown/docs-build:daffy