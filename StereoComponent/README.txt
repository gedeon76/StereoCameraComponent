Author:	Henry Portilla  (C) 2015


This is an small guide about how to use this software component:


The component uses CMake as project generator, in this way you should be able to carry out multiplatform
development, so you will find the following structure:

- A cmake folder: 
		It contains the modules used to find the libraries required to use the component,
		you must be sure that the corresponding libraries are installed on your system

- A src folder: 
		It contains all the source code of the component, all the components try to use standar C++
		code, howewer at the time of writing it uses a mix of C++98 and c++11 favoring the last 
		standard, so you must be sured that your compiler support it.

- An exampleHowtoUse folder:  
		It contains a cmake project example about how to use the library component once you 					have compiled this