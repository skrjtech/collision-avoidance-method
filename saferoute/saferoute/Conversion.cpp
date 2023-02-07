double robX, robY;		//ロボットのグローバル座標[m]
double robRot;			//ロボットの進行方向（グローバル）[rad]
double humanDirect;		//ロボットから見た人間の方向[rad]
double humanDist;		//ロボットから見た人間との距離[m]

double rotRtoD;			//グローバル座標系におけるロボットから見た人間の方向[rad]

double humanX, humanY		//人間のグローバル座標[m]

#define PI 3.141592

int conversion(void){

	//ロボットの現在位置を取得

	//人間のグローバル座標を算出
	


	//人間の方向が第一象限
	if(humanDirect >= 0 && humanDirect >= robRot){
		rotRtoD = rotRot - humanDirect;
		if(rotRtoD > PI){
			rotRtoD = robRtoD - 2 * PI;
		}
		else if(rotRtoD < -1 * PI){
			rotRtoD = rotRtoD + 2 * PI;
		}
		
		humanX = robX + humanDist * cos(rotRtoD);
		humanY = robY + humanDist * sin(rotRtoD);
	}

	else if(humanDirect >= 0 && humanDirect < robRot){
		rotRtoD = humanDirect - robRot;
		if(rotRtoD > PI){
			rotRtoD = robRtoD - 2 * PI;
		}
		else if(rotRtoD < -1 * PI){
			rotRtoD = rotRtoD + 2 * PI;
		}
		
		humanX = robX + humanDist * cos(rotRtoD);
		humanY = robY + humanDist * sin(rotRtoD);
	}

	else{
		rotRtoD = humanDirect + robRot;
		if(rotRtoD > PI){
			rotRtoD = robRtoD - 2 * PI;
		}
		else if(rotRtoD < -1 * PI){
			rotRtoD = rotRtoD + 2 * PI;
		}
		
		humanX = robX + humanDist * cos(rotRtoD);
		humanY = robY + humanDist * sin(rotRtoD);
	}

	return 0;
}
