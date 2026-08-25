
/****************************RIGHT*******************************/
/****************************CAN1*******************************/
//C����ã�R����� R���ϰ벿�ֶ�Ӧû����ֽ�ģ��°벿�֣������ȣ��ȶ�ӦΨһһ������ǩ6�ĵ��
//C��CAN������L��H

#include <math.h>
#include <stdio.h>
#include "CANdata_analysis.h"
#include "chassisR_task.h"
#include "can.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "chassis_power_control.h"
#include "shoot.h"
#define YAW_MOUSE_SEN   0.00005f//0.00005f
#define PITCH_MOUSE_SEN -0.00015f//0.00015f
////reducation of 3508 motor
////m3508����ļ��ٱ�
//#define M3508_MOTOR_REDUCATION 15.764705882f

////m3508 rpm change to chassis speed   576.096  3.14 *07425 = 0.233145
////m3508ת��ת��(rpm)ת���ɵ����ٶ�(m/s)�ı�����c=pi*r/(30*k)��kΪ������ٱ�
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

////m3508 rpm change to motor angular velocity
////m3508ת��ת��(rpm)ת��Ϊ�������ٶ�(rad/s)�ı���
//#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f

////m3508 current change to motor torque
////m3508ת�ص���(-16384~16384)תΪ�ɵ�����ת��(N.m)�ı���
////c=20/16384*0.3��   
//#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f

//rocker value (max 660) change to vertial speed (m/s) 
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.0100f


//�ұ�
float LQR_K_R[12]={       
-11.0224 , -2.664 ,-11.6436,	-20.2754 , 30.2361 , 3.3863,//      �̾��벻��ζ������Ŷ����񶶶�
10.2554 ,  0.9963 ,   1.0764  ,  3.2532 , 40.8928  , 3.5782	
	

};
                                                               
//����
//float Poly_Coefficient[12][4] = {{-611.59342, 639.61534, -247.78576, 7.63222},
//	{-123.40484, 125.69906, -50.31432, 2.10250},
//	{-72.27129, 64.81787, -16.29453, -1.47444},
//	{-239.70338, 215.35520, -54.84512, -4.70619},
//	{67.75831, -7.98603, -62.99347, 39.14809},
//	{52.95899, -46.03012, 5.73408, 4.61071},
//	{789.75527, -675.04834, 201.33612, -6.29698}, 
//	{154.84965, -128.67852, 35.23024, -1.53590},
//	{93.54271, -69.08236, 14.07947, 0.33293},
//{314.08769, -232.28542, 47.78625, 0.89134},
//{-58.19551, -36.72993, 67.63760, 44.63026},
//{-73.61757, 49.79165, -4.25481, 2.91098}};


//	 Q=diag([5 1 10 50 750 1]);
//    R=diag([1 0.25]);
//float Poly_Coefficient[12][4]  = {{-508.13725, 503.69317, -194.12947, 5.56180}, 
//	{-107.79643, 102.09278, -39.47266, 1.59524},
//{	-68.24014, 59.31664, -15.19440, -1.27772}, 
//{	-184.88568, 160.84640, -41.61069, -3.43481},
//{	-189.98743, 181.55909, -92.64975, 34.28558},
//{	23.93030, -20.80343, 0.70362, 4.00147}, 
//{	683.30037,-594.67924, 187.86228, -3.82987},
//{	130.70023,-108.37642, 31.09602, -0.81943},
//{95.40291, -71.44733, 14.77904, 0.84588},
//{259.90459, -194.11221, 40.18588, 2.20853},
//{453.08881, -430.84754, 170.32356, 29.26877}, 
//{-32.52866, 19.03603, 3.18544, 2.06965}};
// Q=diag([5 1 20 50 660 1]);

//R=[1 0;0 0.25];  ʮ���½�Ϊ�ȶ�
//float Poly_Coefficient[12][4] = {{-141.0296,179.7003,-110.7634,0.1796},
//{-24.5119,22.6695,-15.6833,0.2764},
//{-34.6964,38.9717,-16.0315,-3.3770},
//{-27.0710,29.0492,-12.4782,-3.5453},
//{-18.6586,57.3614,-52.0204,21.9498},
//{1.6266,2.2734,-3.9480,2.9919},
//{770.8381,-723.0982,242.0338,-8.2988},
//{101.2459,-86.6996,21.2338,-0.3689},
//{426.6906,-346.5953,89.0961,-7.1877},
//{380.6089,-306.0104,76.9671,-6.4536},
//{-170.1095,41.9958,49.3196,23.0916},
//{-71.7298,47.3327,-5.3442,3.5004},};

	
//float Poly_Coefficient[12][4] = {-421.75465, 404.14066, -163.18492, 1.52481, -100.33598, 88.97437, -33.12351, 0.46394, -13.79006, 20.45913, -5.20346, -6.44626, 0.23096, 8.70198, -1.16290, -7.84332, -145.33854, 158.48464, -89.03226, 32.92067, -27.19789, 19.76695, -10.47246, 4.95830, 615.75677, -496.49431, 155.68024, -1.46787, 129.15010, -93.95712, 23.77051, 0.02808, 207.43929, -152.95834, 33.54335, 1.53506, 211.73999, -148.21559, 29.04610, 2.57854, 88.69753, -151.25099, 93.98502, 36.97904, -21.28958, 10.81717, 4.85406, 2.35990};
////	Q=diag([5 1 60 20 700 1]);
// R=[1 0;0 0.15]; 
//	
//	float Poly_Coefficient[12][4] = {-426.80946, 422.15770, -173.82651, 1.43010, -90.75101, 83.40046, -32.33546, 0.52075, -30.26332, 33.63923, -8.90357, -5.90267, -23.53753, 28.45835, -7.01475, -7.14469, -248.04932, 242.38788, -113.37229, 38.45959, -28.43070, 21.98023, -11.50447, 5.47191, 641.62393, -550.16594, 191.61516, 1.47850, 118.16270, -87.93025, 24.94541, 0.67721, 147.62706, -106.19926, 19.33621, 5.26450, 140.52593, -92.56895, 12.63948, 6.71244, 461.21452, -476.78455, 203.13667, 37.94705, 20.46329, -25.03976, 17.40863, 1.73671};
//
// Q=diag([50 10 100 500 5000 1]);
//float Poly_Coefficient[12][4] = {-1200.43625, 1330.58159, -486.85531, 11.30288, -228.89868, 256.27209, -97.74862, 2.77395, -122.30339, 123.36767, -26.92767, -5.51879, -307.02622, 316.05631, -69.54159, -15.15569, -366.23125, 264.44439, -169.34679, 80.85468, 36.64904, -50.09877, -0.12013, 10.39570, 1269.64177, -1230.22091, 410.92332, -9.10478, 207.09549, -197.15649, 63.54541, -0.41291, 328.66350, -276.00946, 68.57089, -0.24265, 834.06840, -698.59098, 171.97879, 0.30650, 582.13517, -530.52970, 253.14081, 79.76873, -211.73317, 180.55411, -36.36131, 6.28735};


// Q=diag([25 10 100 500 5000 1]);
//float  Poly_Coefficient[12][4] = {-1200.61632, 1331.09538, -487.55065, 11.56030, -229.22825, 256.57954, -97.86707, 2.79580, -124.94753, 125.54078, -27.54556, -5.45484, -317.27402, 324.50489, -71.98316, -14.89156, -359.87726, 259.07761, -167.68584, 80.65459, 39.31541, -52.27830, 0.49725, 10.33233, 1282.45689, -1242.44443, 415.79146, -10.03655, 205.29687, -195.62656, 63.08407, -0.35252, 317.03085, -266.38214, 65.79479, 0.05465, 801.65906, -671.97358, 164.40599, 1.09875, 590.05511, -536.25120, 254.32028, 79.73091, -204.07101, 174.28720, -34.58490, 6.10227};

// Q=diag([25 1 100 500 3000 1]);
// R=[1 0;0 0.25]; 
//float Poly_Coefficient[12][4] = {-1221.64683, 1338.40744, -485.68304, 11.04687, -234.92104, 259.42922, -97.97030, 2.76205, -114.49413, 114.93597, -23.94514, -5.82134, -291.06610, 297.74411, -62.82127, -15.84340, -223.02569, 143.18897, -130.88731, 71.70113, 34.83696, -47.35493, -1.20479, 10.35586, 1210.62300, -1225.76810, 429.20112, -12.88774, 183.21072, -185.05705, 63.33559, -0.58022, 363.31146, -313.20440, 81.94315, -1.60897, 923.99913, -795.35507, 206.92423, -3.35136, 71.45665, -84.70809, 116.12915, 67.70047, -253.81635, 220.55189, -49.85587, 6.94275};

//float Poly_Coefficient[12][4]= {-1035.13068, 1139.76568, -427.16227, 17.97600, -166.50253, 195.90534, -80.33625, 4.15116, -341.99788, 298.51065, -76.25659, -0.06602, -731.78786, 643.40499, -166.36008, -0.18774, 606.03176, -430.88917, -9.66847, 65.78821, 259.96961, -217.86609, 43.12719, 6.75902, 1842.49316, -1685.99114, 517.50351, -12.61764, 328.91133, -303.95906, 90.59383, -3.55641, 338.31457, -245.92555, 39.74519, 5.92439, 756.25802, -556.14875, 94.85121, 11.44057, 193.11676, -443.69489, 335.41586, 40.22730, -206.82416, 124.99174, 4.49564, 0.29165};




//float  Poly_Coefficient[12][4] = {-562.55324, 621.11971, -256.41644, 5.78056, -101.40322, 104.51977, -43.36826, 1.55738, -88.56528, 76.42380, -18.02280, -2.43719, -214.03855, 184.74156, -44.14136, -5.78445, 330.09431, -135.00935, -58.41428, 34.15853, 72.74514, -46.67791, -0.59061, 5.89352, 883.20490, -757.04583, 227.81398, -5.37689, 153.29025, -124.39692, 27.66452, -1.54416, 100.28143, -70.87291, 13.32073, -1.36870, 249.41936, -177.04404, 33.47630, -3.60460, -67.61300, -48.83208, 76.69260, 40.00413, -49.65825, 23.04429, 5.74500, 5.74995};

//float Poly_Coefficient[12][4] = {-504.08027, 547.88046, -229.05457, 6.35126, -85.44726, 91.08219, -40.23197, 1.61730, -97.50615, 86.90826, -22.76244, -1.60305, -227.63908, 203.60564, -53.93584, -3.85791, 110.59586, -32.32819, -63.67274, 42.45456, 62.90033, -49.99657, 4.44254, 6.11821, 1126.01996, -1020.94516, 326.62124, -6.22405, 187.60394, -164.54289, 45.61414, -1.53104, 160.43084, -117.37706, 20.94685, 1.38221, 385.32204, -283.01231, 51.39981, 2.84660, 58.33105, -220.81714, 181.96314, 32.38360, -83.87004, 42.47284, 9.92880, 2.91745};


//float Poly_Coefficient[12][4] = {-530.64231, 551.77861, -222.96182, 5.44010, -87.66597, 88.11793, -37.25990, 1.44378, -96.80539, 85.32716, -22.43996, -1.68445, -198.78774, 175.64756, -46.79873, -3.52435, -33.73268, 84.53282, -84.57247, 37.79028, 31.12601, -22.80255, -0.93371, 4.93553, 996.58249, -889.34115, 278.94744, -4.01454, 158.82573, -136.49888, 37.41964, -1.06478, 140.20748, -101.39122, 17.45934, 1.57632, 295.83186, -214.75459, 37.74723, 2.94917, 277.17318, -355.57380, 186.90267, 30.41472, -26.61540, 3.96811, 13.44307, 2.50917};



//float Poly_Coefficient[12][4] = {-790.61928, 782.26690, -284.57084, 7.42844, -117.89599, 116.14906, -45.60867, 1.97170, -128.23953, 110.73663, -28.62830, -1.05241, -294.42237, 255.24433, -67.05019, -2.29070, -254.16480, 252.82343, -116.41452, 33.85460, 12.40277, -8.05018, -2.74366, 3.37031, 830.45512, -728.15279, 216.14258, -0.02068, 124.03592, -107.56058, 30.99103, -0.52904, 66.74994, -43.96404, 3.29141, 2.96663, 160.80023, -108.06718, 10.23011, 6.43703, 655.93861, -601.09142, 213.79050, 26.98381, 30.72005, -33.34413, 16.23659, 1.46101};


//float Poly_Coefficient[12][4] = {-393.18392, 366.43227, -140.02246, 5.28686, -57.91146, 53.78682, -18.66638, 1.02021, -43.93684, 35.76567, -10.01788, 0.24219, -98.12911, 80.03732, -22.31854, 0.47045, 166.10806, -106.15745, 1.88274, 14.11153, 38.02260, -28.02419, 4.09070, 1.87873, 579.69002, -615.92846, 242.30850, 9.12254, 152.15951, -136.61572, 41.20384, -1.05224, -105.53028, 87.93338, -28.46376, 5.46033, -219.34156, 183.38972, -60.23773, 11.67718, 1932.13769, -1693.14811, 571.31801, -43.46249, 298.68691, -257.26779, 86.56246, -7.85493};

//Q=diag([15 1 70 110 700 1]);
//  R=[50 0;0 4]; 

//float Poly_Coefficient[12][4] = {-351.92258, 339.62900, -136.30113, 3.81699, -58.00549, 54.29040, -19.47503, 0.94701, -33.03773, 27.27440, -7.63558, -0.28915, -72.47616, 59.93984, -16.66305, -0.67372, 3.42468, 27.86047, -39.32608, 19.05945, 11.25948, -6.24780, -2.47818, 2.72622, 510.05106, -530.75297, 197.47805, 6.68507, 123.12547, -109.59115, 30.88059, -0.76736, -83.17181, 69.66464, -23.59165, 4.09415, -163.96735, 138.10345, -47.78112, 8.34158, 1278.09452, -1141.94956, 398.55645, -16.57040, 200.89349, -175.60266, 61.17181, -3.65826};


//Q=diag([15 1 150 70 700 1]);
//  R=[50 0;0 10]; 	
	
//float Poly_Coefficient[12][4] = {-358.59937, 345.68484, -138.44235, 3.90542, -59.19788, 55.56150, -19.99059, 0.97333, -48.63440, 40.06329, -11.24423, -0.41295, -80.37622, 66.41495, -18.44815, -0.73935, 9.44613, 23.01501, -38.47261, 19.16257, 12.90757, -7.60606, -2.18386, 2.74472, 518.41058, -539.80036, 198.93779, 6.71953, 125.13013, -112.18703, 31.45170, -0.77712, -123.50176, 103.24620, -34.93088, 6.05971, -176.15230, 148.36627, -51.74023, 9.05307, 1287.82392, -1151.35130, 402.52745, -17.11427, 203.02862, -177.61902, 62.06144, -3.79480};
//	
//	
	
//float Poly_Coefficient[12][4] = {-241.41391, 269.01932, -126.16534, 0.50746, -64.76692, 62.24131, -21.57308, 0.90627, -13.57549, 11.91142, -2.42283, -1.43818, -29.43940, 25.39531, -5.26027, -2.28636, 35.13890, 20.76799, -58.44971, 18.56599, 6.54585, -0.25212, -7.28612, 2.94768, 663.47360, -613.19214, 186.53674, -1.83203, 118.26811, -99.76021, 23.19437, -1.25124, 34.21463, -23.99967, 1.89139, -0.43644, 64.06889, -45.27344, 4.42703, -1.03602, 190.64083, -239.02467, 110.03181, 21.65076, 19.96229, -27.28585, 14.68923, 3.05332};
//	
//	Q=diag([15 1 70 200 800 1]);
//  R=[60 0;0 20]; 
//float Poly_Coefficient[12][4] = {-252.52185, 275.36335, -126.67940, 0.89703, -61.96643, 58.87512, -20.87352, 0.87078, -7.76607, 6.88055, -1.50170, -0.89801, -23.58152, 20.58366, -4.52029, -2.24385, -26.20899, 68.90493, -66.77975, 20.02921, -2.29562, 6.76497, -8.43935, 3.15219, 625.68246, -580.13186, 181.68363, -0.70856, 117.14806, -98.66189, 23.43106, -1.18522, 12.70335, -7.77778, -0.79912, 0.02318, 38.21801, -24.31969, -0.96443, -0.16640, 236.90723, -274.80354, 124.19567, 19.63241, 29.76325, -34.86428, 17.35134, 2.65218};	
	

//ԭ�����ԵĲ���
//float Poly_Coefficient[12][4] = {-750.37091, 674.65333, -264.83487, 6.61073, -165.70860, 130.33069, -48.34246, 1.99983, -112.41372, 88.27984, -22.36833, -1.81057, -274.12904, 213.96496, -54.59379, -4.26855, 124.07351, 6.68282, -83.93672, 45.00612, 84.42312, -52.97705, 3.26273, 6.17020, 1002.54935, -908.03209, 299.65431, -1.61756, 172.43066, -143.93387, 38.95117, -0.98294, 95.29996, -61.97671, 6.31565, 2.44423, 236.65063, -155.63029, 17.58057, 5.32039, 331.06497, -457.91684, 244.40584, 32.70065, -35.98225, -2.76939, 21.70274, 2.72641};	


//һ�㣬���ǲ�����  ӭ��
float Poly_Coefficient[12][4] = {-421.75465, 404.14066, -163.18492, 1.52481, -100.33598, 88.97437, -33.12351, 0.46394, -13.79006, 20.45913, -5.20346, -6.44626, 0.23096, 8.70198, -1.16290, -7.84332, -145.33854, 158.48464, -89.03226, 32.92067, -27.19789, 19.76695, -10.47246, 4.95830, 615.75677, -496.49431, 155.68024, -1.46787, 129.15010, -93.95712, 23.77051, 0.02808, 207.43929, -152.95834, 33.54335, 1.53506, 211.73999, -148.21559, 29.04610, 2.57854, 88.69753, -151.25099, 93.98502, 36.97904, -21.28958, 10.81717, 4.85406, 2.35990};

//������
//float Poly_Coefficient[12][4] = {-530.56254, 522.04951, -201.35625, 5.81346,
//	-113.71648, 106.71366, -41.22171, 1.68544,
//		-96.25677, 83.18131, -21.24060, -1.83299, 
//		-197.99263, 171.28794, -44.30590, -3.73680,
//		-157.96645, 152.23392, -83.02290, 33.04249,
//		26.54151, -22.98194, 1.25121, 4.06498,
//		659.16889, -580.30264, 187.96273, -3.74313, 
//		125.51825, -104.88944, 30.97526, -0.77725, 
//	127.31478, -96.58071, 20.42665, 1.24595, 
//	264.35282, -199.69374, 42.30404, 2.41148,
//	404.16706, -386.42653, 156.14425, 25.47839, 
//	-33.79435, 20.75503, 2.41196, 1.89930};		


//  Q=diag([5 1 10 100 3000 1]);
//  R=diag([5 1]);   �����е���
//float Poly_Coefficient[12][4] = {-498.60243, 436.80580, -155.44289, 4.73392, -144.51977, 114.63486, -36.21072, 1.70704, -40.52809, 31.27166, -7.52044, -0.60673, -157.19933, 120.85865, -29.19242, -2.10398, -306.30127, 272.76824, -105.61241, 30.10399, 30.59526, -18.69038, -0.10892, 3.14551, 91.86638, -151.94377, 81.03943, 2.24098, 42.83152, -41.95694, 15.13934, -0.15536, -19.32842, 14.80372, -5.04596, 1.49511, -60.67529, 46.68044, -16.51447, 5.27296, 733.62677, -622.95384, 204.31677, 29.26358, 43.30826, -41.00492, 17.76446, 0.91691};
//  Q=diag([5 1 50 300 3000 1]);
//  R=diag([5 1]); ż����Ƶ��
//float Poly_Coefficient[12][4] = {-979.86391, 865.65257, -300.76024, 10.72034, -202.98871, 164.60902, -54.26622, 2.79362, -127.42250, 100.78382, -25.12534, -0.41129, -383.55303, 302.84335, -75.88744, -1.09542, 110.69459, -1.45629, -72.92956, 41.01949, 102.51325, -70.02814, 8.71668, 4.71001, 959.76361, -895.37763, 296.21743, 0.01295, 179.45101, -156.54024, 45.41711, -1.34671, 19.70384, -5.21817, -7.78085, 3.88800, 77.52227, -31.68116, -17.95482, 10.80772, 839.66914, -851.75553, 349.43809, 17.05731, 49.86765, -68.38739, 38.76387, -0.43163};
//  Q=diag([10 1 30 100 700 1]);
//  R=diag([5 1]); ����������Ӧ�е���
//float Poly_Coefficient[12][4] = {-634.71789, 580.54725, -217.50524, 5.83187, -146.13261, 117.46324, -38.36453, 1.83395, -66.63519, 53.13266, -13.46378, -0.98728, -170.26218, 134.74682, -34.08124, -2.35097, 47.29257, 35.84356, -68.08638, 30.68473, 44.96287, -25.71576, -1.32380, 4.25942, 780.34917, -727.73253, 240.80672, 1.63156, 150.56104, -127.10453, 33.30816, -1.00707, 6.27582, 3.82162, -8.46798, 2.63314, 30.53971, -2.83685, -17.28698, 5.87314, 541.44017, -583.48462, 252.13812, 13.43313, 46.52356, -60.96339, 32.43359, 0.74120};

//float Poly_Coefficient[12][4]= {-907.31803, 805.74950, -284.02757, 9.89957, -189.13514, 153.25152, -50.83831, 2.58991, -120.84220, 95.79618, -23.97520, -0.57428, -337.74912, 266.98955, -67.04538, -1.54503, 244.45959, -115.81008, -32.67959, 31.34584, 98.45752, -67.46670, 8.81565, 4.22728, 937.53235, -871.44594, 286.41633, 0.78414, 176.49978, -152.75134, 42.72252, -1.22365, 8.32964, 4.55900, -10.86045, 3.94753, 41.78300, -3.21076, -25.08732, 10.08120, 602.80891, -651.33446, 285.55563, 5.18764, 47.36666, -65.28378, 36.72115, -0.82353};

//  Q=diag([10 1 50 250 700 1]);
//  R=diag([5 1]);
//float Poly_Coefficient[12][4] = {-907.31803, 805.74950, -284.02757, 9.89957, -189.13514, 153.25152, -50.83831, 2.58991, -120.84220, 95.79618, -23.97520, -0.57428, -337.74912, 266.98955, -67.04538, -1.54503, 244.45959, -115.81008, -32.67959, 31.34584, 98.45752, -67.46670, 8.81565, 4.22728, 937.53235, -871.44594, 286.41633, 0.78414, 176.49978, -152.75134, 42.72252, -1.22365, 8.32964, 4.55900, -10.86045, 3.94753, 41.78300, -3.21076, -25.08732, 10.08120, 602.80891, -651.33446, 285.55563, 5.18764, 47.36666, -65.28378, 36.72115, -0.82353};
//	Q=diag([15 1 70 200 800 1]);
//  R=[6 0;0 1]; 
//float Poly_Coefficient[12][4] = {-798.40689, 711.05173, -255.28085, 8.51863, -170.25342, 136.76577, -45.17091, 2.29394, -122.35373, 97.35566, -24.82881, -0.67643, -280.46714, 222.05633, -56.70590, -1.48396, 170.51905, -64.12218, -41.51991, 30.57986, 79.32729, -53.22242, 5.90375, 4.08823, 861.14708, -817.66598, 275.66812, 2.19100, 165.88898, -144.51904, 41.11023, -1.08228, -8.76377, 18.89825, -16.12048, 5.14614, 3.58483, 23.23455, -30.35018, 10.59105, 745.42137, -766.07497, 318.44338, 2.26430, 70.58802, -83.37857, 41.56700, -1.33941};

//911
	
//	float Poly_Coefficient[12][4] = {-811.03039, 726.03218, -260.28106, 8.46041, -172.40958, 139.39647, -46.16507, 2.30459, -85.52958, 67.88088, -17.01499, -0.61528, -271.81466, 215.03275, -54.02877, -1.88160, 163.12831, -53.57502, -46.94561, 31.27005, 77.48402, -51.18043, 4.91249, 4.26818, 868.21742, -809.62892, 268.10684, 1.21585, 164.21792, -141.15263, 38.86507, -1.11304, 4.91743, 4.69889, -8.62011, 2.93673, 30.05325, 2.52301, -23.28827, 8.57566, 594.67665, -636.67708, 275.57933, 7.95729, 49.74987, -65.63044, 35.56727, -0.29459};
//	
//	
//float Poly_Coefficient[12][4] = {-856.71664, 764.01640, -272.93894, 9.28511, -180.06584, 145.51018, -48.52439, 2.48403, -141.73741, 112.68999, -28.38492, -0.92656, -307.63471, 243.45134, -61.54375, -1.88439, 172.01911, -58.62189, -47.40965, 32.35108, 83.81001, -55.94352, 5.95531, 4.36427, 808.12082, -764.80858, 256.11071, 3.01962, 166.25969, -144.44771, 40.40363, -1.21557, -34.76582, 41.44538, -23.12111, 5.76913, -42.47401, 62.03574, -41.31340, 11.06665, 766.03410, -777.07816, 316.23845, 4.54554, 83.44385, -93.16631, 43.57511, -1.16753};
	
	
//	
//float Poly_Coefficient[12][4] =	{-188.752964070179,	243.102124784253,	-152.644559878110,	1.34952904316168,
//4.1746585131290,	-2.30106979988108	,-11.9653380830797	,0.194490644557998 ,
//-35.40720950655,	32.1411493870097	,-10.1728757283575	,-2.02169951220151 ,
//-64.03129929349,	57.9323334414574	,-19.0712288031785	,-4.19105845966687 ,
//-66.52876710298,	135.364778312744	,-99.0078734879657	,35.8878982807632  ,
//1.8225591754489,	8.20377303627651	,-9.76087901716572	,5.56226664583508  ,
//685.49757844125,	-634.093737655786	,212.664891058675	,8.96386652889417  ,
//50.287685610320,	-48.3483171488855	,13.7412345200768	,0.461778151427116 ,
//-66.03468699868,	77.2417541895691	,-35.6512929287317	,6.40860244824254  ,
//-129.6788170966,	150.590270007289	,-69.2247487418621	,12.0623335908354  ,
//1106.0049074416,	-1102.52032910889	,419.708104798520	,-1.34851146322992 ,
//166.40467543627,	-169.741074517090	,66.9447774767495	,-2.18342410694397 };
	
	
	
	
	vmc_leg_t right;//����

extern INS_t INS;

extern vmc_leg_t left;

chassis_t chassis_move_balance;
extern c_fbpara_t  C_data;

float jump_time_r;
extern float jump_time_l;
												
pid_type_def LegR_Pid;//���ȵ��ȳ�pd

pid_type_def Tp_Pid;//�����油��pd
pid_type_def Turn_Pid;//ת��pd
pid_type_def Roll_Pid;//����ǲ���pd
pid_type_def Wheel_Pid; //3508PID
pid_type_def Wz_Pid;
extern pid_type_def buffer_pid;

extern shoot_control_t shoot_control;          //�������

/**
* @description: �����ں�
* @param {dt} ʱ�䲽��
* @param {acc} ���ٶȲ���ֵ
* @param {speed} �ٶȲ���ֵ
* @param {vel_esti} ָ������ٶȵ�ָ��
* @return {*}
*/
float Rz = 1; 
float Qz = 0.03;

void speed_est(const float dt, const float acc, const float speed, float *vel_esti)
{
	static float _x = 0;
	static float _Pz = 1;

	float H = 1;
	float A = 1;
	float B = dt;
	float _x_est;
	float _p_est;
	float K;

	// Ԥ�ⲽ��
	_x_est = A * _x + B * acc;
	_p_est = _Pz + Qz; // A * _P * A_T + Q

	// ���²���
	K = _p_est / (_p_est + Rz);
	_x = _x_est + K * (speed - _x_est);
	_Pz = (1 - K) * _p_est;   

	*vel_esti = _x;
}
// ��ʼ����̬����
//static float Rz = 1.0f;       // ��ʼ������������
//static float Qz = 0.03f;      // �̶�������������
//static float _x = 0.0f;       // ״̬����
//static float _Pz = 1.0f;      // ����Э����

//void speed_est(const float dt, const float acc, const float speed, float *vel_esti) {
//    const float ALPHA = 0.03f; // ƽ��ϵ�� (5%)
//    const float MAX_ERR = 3.5f; // ��������в�

//    float H = 1.0f, A = 1.0f, B = dt;
//    float _x_est, _p_est, K;

//    // Ԥ�ⲽ��
//    _x_est = A * _x + B * acc;
//    _p_est = _Pz + Qz;

//    // ��̬����Rz
//    float residual = speed - _x_est;
//    residual = fmaxf(fminf(residual, MAX_ERR), -MAX_ERR); // �޷�
//    Rz = (1 - ALPHA) * Rz + ALPHA * residual * residual;

//    // ���²���
//    K = _p_est / (_p_est + Rz);
//    _x = _x_est + K * residual;
//    _Pz = (1 - K) * _p_est;

//    *vel_esti = _x;
//}

extern float LQR_K_L[12];
float kf_fusion;
float forward_acc;

float data_fusion(void)//�ٶ��ںϵ���
	{
	float fusion;
	forward_acc = -INS.MotionAccel_b[0];
	float speed;
	//�ٶȷ��� �޸�
	//	chassis->v = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;	

	speed = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;
	
	speed_est(0.001,forward_acc,speed,&fusion);

	return fusion;
}


uint32_t CHASSR_TIME=1;	


float yaw_sen;
float pitch_sen;
float mode_rc;
float fire_mode;

void ChassisR_task(void)
{

	chassis_move_balance.leg_set=0.13; 
	chassis_move_balance.turn_set = 0;
	chassis_move_balance.x_set = 0;
	chassis_move_balance.v_set = 0;
	chassis_move_balance.recover_flag = 0;
	chassis_move_balance.roll_set = 0;
	shoot_control.shoot_send_flag = 0;
	
	while(INS.ins_flag==0)
	{//�ȴ����ٶ�����
	  osDelay(1);	
	}

	
	//�����ʼ��
	//�������ұ߹ؽڵ����id,mode��ʼ�����Լ������ʹ�ܣ�
	  ChassisR_init(&chassis_move_balance,&right,&LegR_Pid,&Wheel_Pid);
	  Pensation_init(&Roll_Pid,&Tp_Pid,&Turn_Pid,&Wz_Pid);//����pid��ʼ��
		
	  shoot_init();
    //chassis_move_balance.leg_set = 0.127f;//ԭʼ�ȳ�    �ȳ�������0.127------0.32  ����趨��1-2cm
	
	
	while(1)
	{

		
		chassis_move_balance.DUBS_ON=toe_is_error(DBUS_TOE);//ң����������0����������1
		
//		c_transmit_date(&hcan1,chassis_move_balance.chassis_RC->rc.s[0],
//		chassis_move_balance.chassis_RC->rc.ch[3],
//		chassis_move_balance.chassis_RC->rc.ch[2],
//		chassis_move_balance.chassis_RC->rc.s[1],
//		INS.Yaw,chassis_move_balance.chassis_RC->key.v,
//		chassis_move_balance.DUBS_ON);
		
		if(chassis_move_balance.chassis_RC->rc.s[1] == 1){
		
		fire_mode = 1;
		
		}
		if(chassis_move_balance.chassis_RC->rc.s[1] == 3){
		
		fire_mode = 0;
		
		}
		
		//*************************************************************
	    c_transmit_date(yaw_sen, pitch_sen,mode_rc,shoot_control.shoot_send_flag, 11);
		
		osDelay(1);
		
		
		float dt = (float)CHASSR_TIME/1000.0f;	
		
		if( chassis_move_balance.chassis_RC->rc.s[0] == 3)
		{
		   chassis_move_balance.start_flag=1;
			mode_rc  = 1; 
		}
		
			
		if( chassis_move_balance.chassis_RC->rc.s[0] == 2)
		{
			
		   chassis_move_balance.start_flag=0;
			mode_rc  = 0;   //����ģʽ
		}

		if( chassis_move_balance.chassis_RC->rc.s[0] == 1)
		{
			chassis_move_balance.w_flag  = 1;//���̲�������̨

		}else{
		chassis_move_balance.w_flag  = 0;//���̸�����̨
		}
		
		
	    chassis_move_balance.recover_flag = recover_detect(&chassis_move_balance);	

	if(chassis_move_balance.start_flag==1 )
   {
	
	   if(chassis_move_balance.chassis_RC->key.v & CHASSIS_FRONT_KEY){
	
	        chassis_move_balance.target_v = 2;
	
	    }else if(chassis_move_balance.chassis_RC->key.v & CHASSIS_BACK_KEY){
			chassis_move_balance.target_v = -2;

		
	     }
		
		chassis_move_balance.target_v=((float)chassis_move_balance.chassis_RC->rc.ch[1])*(0.0050f);//��ǰ����0	
		slope_following(&chassis_move_balance.target_v,&chassis_move_balance.v_set,0.004f);	//	�¶ȸ���
			
//	    chassis_move_balance.v_set=((float)chassis_move_balance.chassis_RC->rc.ch[1])*(0.00450f);//��ǰ����0	
		chassis_move_balance.x_set = chassis_move_balance.x_set+chassis_move_balance.v_set*(float)CHASSR_TIME*2.0f/1000.0f;
	    
		yaw_sen = chassis_move_balance.chassis_RC->rc.ch[2]*(0.00003f) - chassis_move_balance.chassis_RC->mouse.x * YAW_MOUSE_SEN;//���Ҵ���0
	
		pitch_sen =((float)chassis_move_balance.chassis_RC->rc.ch[3])*(0.00003f) + chassis_move_balance.chassis_RC->mouse.y * PITCH_MOUSE_SEN;
	
//       chassis_move_balance.turn_set = chassis_move_balance.turn_set+(float)chassis_move_balance.chassis_RC->rc.ch[2]*(-0.00006f);

	///���Ҵ���0		
	if((shoot_control.shoot_rc->key.v & CHASSIS_LEG_KEY) && 
        !(shoot_control.last_key & CHASSIS_LEG_KEY)){
	
	   	chassis_move_balance.leg_set = chassis_move_balance.leg_set + 0.1; 

	
	}
   	chassis_move_balance.leg_set = chassis_move_balance.leg_set+(((float)chassis_move_balance.chassis_RC->rc.ch[0])*(0.0000037f)); 

//
		mySaturate(&chassis_move_balance.leg_set,0.130f,0.32f);
		
		if(fabsf(chassis_move_balance.last_leg_set-chassis_move_balance.leg_set)>0.0007f)
	{
				//ң���������ȳ��ڱ仯
				right.leg_flag=1;	//Ϊ1��־��ң�����ڿ����ȳ����������������־���Բ�������ؼ�⣬��Ϊ���ȳ�����������ʱ����ؼ�������Ϊ�����
				left.leg_flag=1;	 			
}
	
	chassis_move_balance.last_leg_set=chassis_move_balance.leg_set;
	  
	if(chassis_move_balance.w_flag == 1)  //С����
	{
		chassis_move_balance.Wz_set=1;
		chassis_move_balance.v_set = 0;
		chassis_move_balance.x_set = 0;
		
	}else {
	chassis_move_balance.w_flag=0;
	
	chassis_move_balance.Wz_set=0;
	}

			
}
		
		//��������
	    chassisR_feedback_update(&chassis_move_balance,&right,&INS);
		
//	chassis_move_balance.turn_set = 0;

//	 chassis_set_mode(&chassis_move_balance);

	//���Ƽ���
	    chassisR_control_loop(&chassis_move_balance,&right,&INS,LQR_K_L,&LegR_Pid);	
//		mit_ctrl(&hcan1,0X06,0,1,0,1,0);



		if(chassis_move_balance.start_flag==1)	
		{
			mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.7f,right.torque_set[1]);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.7f,right.torque_set[0]);//right.torque_set[0]
			osDelay(CHASSR_TIME);	
			//˳ʱ��Ϊ��
			chassis_move_balance.wheel_motor[0].given_current = (chassis_move_balance.wheel_motor[0].wheel_T/0.000396211f);
    	    CAN_cmd_chassis(-chassis_move_balance.wheel_motor[0].given_current);
            osDelay(CHASSR_TIME);

//			mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
//			osDelay(CHASSR_TIME);
//			mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
//			osDelay(CHASSR_TIME);
//			CAN_cmd_chassis(0);
			

		}

		
		 else if(chassis_move_balance.start_flag==0)	
		{
			mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.7f,0.0f);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.7f,0.0f);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			CAN_cmd_chassis(0);
			osDelay(CHASSR_TIME);
		}
  }
}





//���̳�ʼ��
void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legr,pid_type_def *wheel)
{
	
	chassis->chassis_RC = get_remote_control_point();
  const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
  
  const static fp32 power_pid[3] = {POWER_PID_KP, POWER_PID_KI, POWER_PID_KD};
  
  const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	//�ұ߹ؽڵ���ĳ�ʼ��
	joint_motor_init(&chassis->joint_motor[0],6,MIT_MODE);//����idΪ6
	joint_motor_init(&chassis->joint_motor[1],8,MIT_MODE);//����idΪ8
	
	VMC_init(vmc);//���˳���ֵ
	//�ȳ�pid��ʼ��
	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	  osDelay(1);
	}
	
	    for (int m = 0; m < 2; m++)
    {
       
       PID_init(&chassis->motor_speed_pid[m], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
	
	
	PID_init(&chassis->buffer_pid, PID_POSITION,power_pid,POWER_PID_MAX_OUT, POWER_PID_MAX_IOUT);
}


float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
float wz_pid[3] = {WZ_PID_KP, WZ_PID_KI, WZ_PID_KD};
//PID��ʼ��
void Pensation_init(pid_type_def *roll,pid_type_def *Tp,pid_type_def *turn,pid_type_def *wz)
{//����pid��ʼ��������ǲ����������油����ƫ���ǲ���
    

//	const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
//	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
//	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
//	const static fp32 power_pid[3] = {POWER_PID_KP, POWER_PID_KI, POWER_PID_KD};

//	����ǲ���
	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	//�����油��
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	//ƫ���ǲ���
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
	PID_init(wz, PID_POSITION, wz_pid, WZ_PID_MAX_OUT, WZ_PID_MAX_IOUT);

//	PID_init(&buffer_pid, PID_POSITION,power_pid,POWER_PID_MAX_OUT, POWER_PID_MAX_IOUT);

}


//�������ݷ�������
void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
	
	//get remote control point
    //���û�ȡң����ָ�룬�õ���ǰ����ͨ����ֵ  
    chassis->chassis_RC = get_remote_control_point();
	
    vmc->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[1].para.pos;
		
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[1];
	
	chassis->relative_angle = chassis->yaw_motor_angle;
	
	chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;
	chassis->theta_err=0.0f-(vmc->theta+left.theta);
	
	chassis->wheel_motor[0].vel = chassis->wheel_motor[0].speed_rpm * CHASSIS_MOTOR_RPM_TO_OMG_SEN;  //���ٶ�
	chassis->wheel_motor[0].speed = 0.0004998609952f * chassis->wheel_motor[0].speed_rpm;  //�ٶ�
//	chassis->wheel_motor[0].wheel_T = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * chassis->wheel_motor[0].given_current;

//	chassis->Wz = (chassis->wheel_motor[0].vel)+(chassis->wheel_motor[1].vel )/2;
//chassis->Wz = chassis->total_yaw;

//	chassis->v = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;	
//		
//	chassis->x = chassis->x + chassis->v*((float)1/1000.0f);
//	
	
	chassis->v = data_fusion();
	chassis->x = chassis->x + chassis->v*((float)CHASSR_TIME/1000.0f);
	
//	
	//����pitch�Ƕ��жϵ��������Ƿ����
	if(ins->Pitch<(3.1415926f/15.5f)&&ins->Pitch>(-3.1415926f/15.5f))
	{
		chassis->recover_flag=0;
	}

	
}


void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==8)
	{//���ص�������8���ֽ�
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}
	


uint8_t right_flag=0;

extern uint8_t left_flag;

int jump_right_flag = 0;


void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,pid_type_def *leg)
{
	
	VMC_calc_1_right(vmcr,ins,((float)CHASSR_TIME)*3.0f/1000.0f);//����theta��d_theta��lqr�ã�ͬʱҲ�������ȳ�L0,���������������3*0.001��

    for(int i=0;i<12;i++)
   {
	LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0);	
   }

//	//chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw��pid����
//  chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];//�����������һ��
//	//chassis->roll_f0=PID_Calc(&Roll_Pid, chassis->roll,chassis->roll_set);//roll��pid����
//	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[1];
//	chassis->leg_tp=PID_Calc(&Tp_Pid, chassis->theta_err,0.0f);//������pid����
	
   
   if(chassis->w_flag==1){
	
	   chassis->turn_T=Turn_Pid.Kp*(chassis->Wz_set-0)-Turn_Pid.Kd*ins->Gyro[2];//�����������һ��

	}
	else{
		chassis->turn_T=Turn_Pid.Kp*(chassis->relative_angle-0)-Turn_Pid.Kd*ins->Gyro[2];//�����������һ��
      //chassis->turn_T = - PID_calc(&Turn_Pid,chassis->relative_angle,0);
	
	}
	
	
	//Roll�Ჹ��
	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[0];
	//Roll�Ჹ����pid�޷�
	mySaturate(&chassis->roll_f0,-Roll_Pid.max_out,Roll_Pid.max_out);
	//������pid����
	chassis->leg_tp=PID_calc(&Tp_Pid, chassis->theta_err,0.00f);
	
//��챵��
//chassis->wheel_motor[0].wheel_T = (LQR_K[0]*(vmcr->theta-0.0f)
//+LQR_K[1]*(vmcr->d_theta-0.0f)
//+LQR_K[2]*(chassis->x-chassis->x_set)
//+LQR_K[3]*(chassis->v-chassis->v_set)
////+LQR_K[2]*(chassis->x_filter-chassis->x_set)
////+LQR_K[3]*(chassis->v_filter2-0.4f*chassis->v_set)
//+LQR_K[4]*(chassis->myPithR-0.01025f) 
//+LQR_K[5]*(chassis->myPithGyroR-0.01f));	

////chassis->wheel_motor[0].wheel_T= ( LQR_K[3]*(chassis->v-0.4f*chassis->v_set) - LQR_K[4]*(chassis->myPithR-0.04f-chassis->phi_set) - LQR_K[5]*(chassis->myPithGyroR-0.0f));		


////�ұ��Źؽ��������				

//vmcr->Tp= (LQR_K[6]*0.8*(vmcr->theta-0.0f)	
//+LQR_K[7]*0.7*(vmcr->d_theta-0.0f)
//+LQR_K[8]*(chassis->x-chassis->x_set)
//+LQR_K[9]*(chassis->v-chassis->v_set)
////+LQR_K[8]*(chassis->x_filter-chassis->x_set)
////+LQR_K[9]*(chassis->v_filter2-0.4f*chassis->v_set)
//+LQR_K[10]*(chassis->myPithR-0.01025f)
//+LQR_K[11]*(chassis->myPithGyroR-0.01f));


	//��챵��
	chassis->wheel_motor[0].wheel_T = (LQR_K[0]*(vmcr->theta-0.0f)
									    +LQR_K[1]*(vmcr->d_theta-0.0f)
//										+LQR_K[2]*(chassis->x-chassis->x_set)
//										+LQR_K[3]*(chassis->v-chassis->v_set)
									    +LQR_K[2]*(chassis->x_filter-(chassis->x_set))
									    +LQR_K[3]*(chassis->v_filter2-(chassis->v_set))
//									    +LQR_K[4]*(chassis->myPithR-0.01025f-chassis->phi_set) 
										+LQR_K[4]*(chassis->myPithR-0.0f) 
									    +LQR_K[5]*(chassis->myPithGyroR-0.0f));	
	
	//�ұ��Źؽ��������				
	  vmcr->Tp = (LQR_K[6]*(vmcr->theta-0.0f)	
				 +LQR_K[7]*(vmcr->d_theta-0.0f)
//	             +LQR_K[8]*(chassis->x-chassis->x_set)
//	             +LQR_K[9]*(chassis->v-chassis->v_set)
				 +LQR_K[8]*(chassis->x_filter-(chassis->x_set))
				 +LQR_K[9]*(chassis->v_filter2-chassis->v_set)
//			     +LQR_K[10]*(chassis->myPithR-0.01025f-chassis->phi_set)
				 +LQR_K[10]*(chassis->myPithR-0.0f)
				 +LQR_K[11]*(chassis->myPithGyroR-0.0f));
		
		chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T - chassis->turn_T;	//��챵���������	
	

//����챵������޷�
	mySaturate(&chassis->wheel_motor[0].wheel_T,-4.2f,4.2f);
	
	vmcr->Tp=vmcr->Tp+chassis->leg_tp;//�Źؽ��������
	vmcr->F0=17.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,chassis->leg_set)-chassis->roll_f0;//ǰ��+pd
	
	
//	if(chassis->chassis_RC->rc.s[1] ==1)
//	{
//		if(chassis->chassis_RC->rc.ch[4] >500){
//		
//		    chassis->help_jump_flag =1;
//		
//		   }
//		
//		  if(chassis->jump_flag_r==0 && chassis->help_jump_flag ==1){
//			//ѹ���׶�
//		      chassis->leg_set = 0.130;

//		       if(vmcr->L0<0.17f)
//		     {
//		        jump_time_r++;  
//		      }
//		     if(jump_time_r>=10&&jump_time_l>=10)
//		     {  
//			   jump_time_r=0;
//			   jump_time_l=0;
//			   chassis->jump_flag_r=1;//ѹ����Ͻ����������ٽ׶�
//			   chassis->jump_flag_l=1;//ѹ����Ͻ����������ٽ׶�
//		     }			 
//		   }
//		
//		else if(chassis->jump_flag_r==1&& chassis->help_jump_flag ==1)
//		{//�������ٽ׶�			
//			chassis->leg_set = 0.30;

//			 if(vmcr->L0>0.22f)
//			 {
//				jump_time_r++;
//			 }
//			 if(jump_time_r>=10&&jump_time_l>=10)
//			 {  
//				 jump_time_r=0;
//				 jump_time_l=0;
//				 chassis->jump_flag_l=2;//������Ͻ������Ƚ׶�
//				 chassis->jump_flag_r=2;
//			 }	 
//		}
//		
//	 else if(chassis->jump_flag_r==2&& chassis->help_jump_flag ==1)
//		{//���Ƚ׶�
//			chassis->leg_set = 0.13;
//			chassis->theta_set=0.0f;
//			
//			chassis->x_filter=0.0f;
//			chassis->x_set=chassis->x_filter;
//			
//		  if(vmcr->L0<0.17f)
//		  {
//			 jump_time_r++;
//		  }
//		  if(jump_time_r>=5&&jump_time_l>=5)
//		  { 
//			 jump_time_r=0;
//			 jump_time_l=0;
//			 chassis->leg_set=0.130f;
//			 chassis->last_leg_set=0.130f;
//			 chassis->jump_flag_r=0;//�������
//		   chassis->jump_flag_l=0;
//       chassis->help_jump_flag = 0;			  
//		  }
//		}
//		
//	else
//	{
//		vmcr->F0=11.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,chassis->leg_set);//ǰ��+pd
//	}
//}	

   right_flag = ground_detectionR(vmcr,ins);//������ؼ��
	 
	 if(chassis->recover_flag==0)		
	 {//����������Ҫ����Ƿ����	 
//		if((right_flag==1&&left_flag==1&&vmcr->leg_flag==0&&chassis->jump_flag!=1&&chassis->jump_flag2!=1&&chassis->jump_flag!=2&&chassis->jump_flag2!=2)
//			||chassis->jump_flag==3)
		if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
		{ 
			//������ͬʱ��ز���ң����û���ڿ����ȵ�����ʱ������Ϊ���
			//�ų���Ծ��ѹ���׶Ρ������׶Ρ���Ծ�����Ƚ׶�
				chassis->wheel_motor[0].wheel_T=0.0f;
				vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

				chassis->x_filter=0.0f;
				chassis->x_set = chassis->x_filter;
				vmcr->Tp=vmcr->Tp+chassis->leg_tp;			 
		}
		else
		{//û�����
			vmcr->leg_flag=0;//��Ϊ0
							
//			if(chassis->jump_flag==0)
//			{//����Ծ��ʱ����Ҫroll�Ჹ��						
//			 vmcr->F0=vmcr->F0+chassis->roll_f0;//roll�Ჹ��ȡ��Ȼ�����ȥ    			
//			}
		}
	 }
	 else if(chassis->recover_flag==1)

	 {
		 vmcr->Tp=0.0f;
		 vmcr->F0=0.0f;
	 }

    //���ʿ���
    chassis_power_control(chassis);

	mySaturate(&vmcr->F0,-100.0f,100.0f);//�޷� 

	VMC_calc_2(vmcr);//���������Ĺؽ��������

//�޷�����������Ҫ�Ĳ���
	mySaturate(&vmcr->torque_set[1],-18.0f,18.0f);	
	mySaturate(&vmcr->torque_set[0],-18.0f,18.0f);	
	 
	 
}
	

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}



uint8_t recover_detect(chassis_t *chassis)
{	
	if(((chassis->myPithR<((-3.1415926f)/10.5f)&&chassis->myPithR>((-3.1415926f)/2.0f))
					  ||(chassis->myPithR>(3.1415926f/12.8f)&&chassis->myPithR<(3.1415926f/2.0f))))	
	{
				
			chassis->leg_set = 0.130;
	
				return 1;//��Ҫ����	
		}

	else
	  {
		  
	  return 0;	
		  
	   }
}

