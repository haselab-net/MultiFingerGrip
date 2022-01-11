/*
 *  Copyright (c) 2003-2012, Shoichi Hasegawa and Springhead development team 
 *  All rights reserved.
 *  This software is free software. You can freely use, distribute and modify this 
 *  software. Please deal with this software under one of the following licenses: 
 *  This license itself, Boost Software License, The MIT License, The BSD License.   
 */

#include "MultiFinger.h"
#include <conio.h>


#pragma hdrstop		// プリコンパイル済ヘッダを利用する場合に必要
MultiFinger app;

int __cdecl main(int argc, char *argv[]){

	app.Init(argc, argv);
	app.StartMainLoop();
	return 0;
}
