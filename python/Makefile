dummy:
	@echo 'Use make [wrapper|build|install]'

clean:
	@rm -rf build

wrapper:
	@swig3.0 -python -c++ -o CSF/CSF_wrap.cxx CSF/CSF.i

build: wrapper
	@python setup.py build

install: build
	@python setup.py install
