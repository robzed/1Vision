/*
 *  Zw_calculate.cpp
 *  red_extract_cocoa
 *
 *  Created by Rob Probin on 01/06/2008.
 *  Copyright (C) 2008 Rob Probin.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#include "Zw_calculate.h"
#include "Real_World.h"
#include "sine_cosine.h"

#if !defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)
static int Yp_to_Zw_conversion_table[processed_image_height_in_pixels]= {	
	146351, // 32519
	147073, // 91388
	147801, // 3583
	148533, // 70758
	149271, // 01149
	150013, // 32047
	150760, // 68567
	151513, // 15892
	152270, // 79278
	153033, // 6405
	153801, // 7561
	154575, // 19434
	155354, // 01073
	156138, // 26156
	156928, // 00392
	157723, // 29568
	158524, // 19554
	159330, // 76304
	160143, // 05854
	160961, // 14329
	161785, // 07939
	162614, // 92985
	163450, // 75857
	164292, // 6304
	165140, // 61111
	165994, // 76744
	166855, // 16708
	167721, // 87875
	168594, // 97217
	169474, // 51806
	170360, // 58824
	171253, // 25556
	172152, // 59398
	173058, // 67856
	173971, // 58549
	174891, // 39212
	175818, // 17696
	176752, // 01972
	177693, // 00135
	178641, // 20402
	179596, // 71116
	180559, // 60751
	181529, // 97911
	182507, // 91336
	183493, // 499
	184486, // 8262
	185487, // 98651
	186497, // 07297
	187514, // 18007
	188539, // 40382
	189572, // 84177
	190614, // 59303
	191664, // 75832
	192723, // 44
	193790, // 74208
	194866, // 77029
	195951, // 63206
	197045, // 43664
	198148, // 29506
	199260, // 32019
	200381, // 6268
	201512, // 33158
	202652, // 55319
	203802, // 41227
	204962, // 03154
	206131, // 53579
	207311, // 05197
	208500, // 70919
	209700, // 63879
	210910, // 97441
	212131, // 85199
	213363, // 40987
	214605, // 78881
	215859, // 13207
	217123, // 58542
	218399, // 29726
	219686, // 41862
	220985, // 10324
	222295, // 50767
	223617, // 79126
	224952, // 11628
	226298, // 64796
	227657, // 55459
	229029, // 00753
	230413, // 18135
	231810, // 25387
	233220, // 40622
	234643, // 82295
	236080, // 69211
	237531, // 2053
	238995, // 55779
	240473, // 94859
	241966, // 58055
	243473, // 66042
	244995, // 39901
	246532, // 01123
	248083, // 71622
	249650, // 73744
	251233, // 30277
	252831, // 64467
	254446, // 00022
	256076, // 61131
	257723, // 72468
	259387, // 59214
	261068, // 47061
	262766, // 62232
	264482, // 31488
	266215, // 82151
	267967, // 42108
	269737, // 39834
	271526, // 04405
	273333, // 65513
	275160, // 53481
	277006, // 99286
	278873, // 34568
	280759, // 91657
	282667, // 03583
	284595, // 04103
	286544, // 27715
	288515, // 09683
	290507, // 86055
	292522, // 93685
	294560, // 7026
	296621, // 54318
	298705, // 85277
	300814, // 03455
	302946, // 50102
	305103, // 6742
	307285, // 986
	309493, // 8784
	311727, // 80384
	313988, // 22547
	316275, // 61751
	318590, // 46556
	320933, // 26694
	323304, // 53105
	325704, // 77977
	328134, // 54778
	330594, // 38301
	333084, // 84705
	335606, // 51555
	338159, // 97866
	340745, // 84153
	343364, // 72476
	346017, // 26489
	348704, // 11493
	351425, // 94492
	354183, // 44242
	356977, // 31315
	359808, // 28157
	362677, // 09152
	365584, // 50682
	368531, // 31203
	371518, // 31309
	374546, // 33809
	377616, // 23802
	380728, // 88756
	383885, // 18593
	387086, // 05775
	390332, // 45393
	393625, // 35265
	396965, // 76028
	400354, // 71246
	403793, // 27515
	407282, // 54575
	410823, // 65425
	414417, // 76449
	418066, // 07541
	421769, // 82237
	425530, // 27856
	429348, // 75648
	433226, // 60942
	437165, // 23312
	441166, // 06737
	445230, // 59785
	449360, // 35788
	453556, // 93042
	457821, // 95008
	462157, // 10518
	466564, // 14009
	471044, // 85747
	475601, // 12076
	480234, // 8568
	484948, // 0585
	489742, // 78772
	494621, // 17827
	499585, // 43909
	504637, // 85755
	509780, // 80301
	515016, // 73051
	520348, // 18465
	525777, // 80374
	531308, // 32416
	536942, // 58494
	542683, // 53262
	548534, // 22641
	554497, // 84359
	560577, // 6853
	566777, // 18261
	573099, // 90298
	579549, // 55713
	586130, // 00626
	592845, // 26978
	599699, // 53351
	606697, // 15836
	613842, // 68959
	621140, // 86667
	628596, // 63372
	636215, // 15074
	644001, // 80545
	651962, // 22608
	660102, // 29486
	668428, // 16255
	676946, // 26394
	685663, // 33444
	694586, // 42783
	703722, // 93529
	713080, // 60585
	722667, // 56828
	732492, // 35464
	742563, // 92563
	752891, // 69784
	763485, // 5731
	774355, // 97017
	785513, // 85887
	796970, // 79709
	808738, // 97063
	820831, // 23651
	833261, // 16977
	846043, // 11434
	859192, // 23825
	872724, // 59368
	886657, // 18233
	901008, // 02666
	915796, // 24771
	931042, // 15008
	946767, // 3149
	962994, // 70184
	979748, // 7609
	997055, // 55533	
};
#else
static short Yp_to_Zw_conversion_table[processed_image_height_in_pixels];
#endif

#if !defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)
int get_Zw_in_mm(int Yp_in_pixels_from_top)
{
	return Yp_to_Zw_conversion_table[239-Yp_in_pixels_from_top]/1000;
}

#include <iostream>

#else
int get_Zw_in_mm(int Yp_in_pixels_from_top)
{
	return Yp_to_Zw_conversion_table[239-Yp_in_pixels_from_top];
}
#endif



//
// This allows us to update the table, when the Real world 'constants' are changed
//
void update_Zw_table()
{
#if !defined(WORLD_RELATED_CONSTANTS_AS_CONSTANTS) && !defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)

	double Yp_pixels = (-(processed_image_height_in_pixels/2.0))+0.5;
	for(int i = 0; i<processed_image_height_in_pixels; i++)
	{
		double Yp_mm = Yp_pixels * pixel_size_in_mm;
		double Yw_mm = wallHeightfromCamera_in_mm;
		
		double D = raw_image_to_eyepoint_in_mm;
		
		//double CosB = SinA;
		//double SinB = CosA;
		double CosB = CosA;
		double SinB = SinA;
		
		double top = Yp_mm * D - D * (-Yw_mm) * CosB - Yp_mm * (-Yw_mm) * SinB;
		double bottom = D * SinB - Yp_mm * CosB;
		double Zw_mm = top / bottom;
		Yp_to_Zw_conversion_table[i] = Zw_mm * 1000;
		
		std::cout << Yp_pixels << " " << Yp_mm << " " << CosA << " " << SinA << " " << top << " " << bottom << " " << Zw_mm  << " " << i << " " << Yp_to_Zw_conversion_table[i] << std::endl;
		
		Yp_pixels += 1.0;
	}
	
#elif defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)
	int Yp_pixels_times_two = (-processed_image_height_in_pixels)+1;
	const int Ypp_scale = 2;
	for(int i = 0; i<processed_image_height_in_pixels; i++)
	{
		int Yp_xm = Yp_pixels_times_two * pixel_size_in_xm / Ypp_scale;			// /2 compensates for the Yp*2		max = 241 * 152 / 2 = 18316 (scaled by 10,000)
		int Yw_mm = wallHeightfromCamera_in_mm;									// max = 135 (scaled by 1)
		
		int D_um = raw_image_to_eyepoint_in_um;		// probably about 5867 as um (scaled by 1000)

		int top1 = Yp_xm * D_um;						// max = 18316 * 5867 = 107,459,972  (scaled by 10,000, and 1000)
		int top2 = D_um * (-Yw_mm) * CosA_scaled;		// max = 5867 * -135 * -/+1000 = -/+792,045,000 (scaled by 1000, 1, 1000)
		// top3 has a problem with overflow without intermediate scaling
		int top3_intermediate = Yp_xm * (-Yw_mm);		// max = 18316 * -135 = -2,472,660  (scaled by 10,000 and 1)
		top3_intermediate /= 10;					// max = -247,266 (scaled by 1000)
		int top3 = top3_intermediate * SinA_scaled;		// max = -247,266 * -/+1000 = -/+247,266,000 (scaled by 1000 1 1000). Maxint = 2,147,483,647 to 2,147,483,647
		// get into same units before adding/subtracting!
		top1 /= 10;			// max = 10,745,997 (scaled by 1,000,000)
		
		int top = top1 - top2 - top3;					// max = 107,459,972 + 792,045,000 + 247,266,000 = 1,146,770,972 (scaled by 1,000,000)
		int bottom = D_um * SinA_scaled - (Yp_xm * CosA_scaled)/10;	// max = 5867 * -/+1000  - (18316 * (-/+1000))/10 = 24,182,000  (scaled by 1,000,000)
		int Zw_mm = top / bottom;
		Yp_to_Zw_conversion_table[i] = Zw_mm;
		
		//std::cout << "Zw INT: " << Yp_pixels_times_two << " Yp_xm=" << Yp_xm << " " << CosA_scaled << " " << SinA_scaled << " " << top << " " << bottom << " " << Zw_mm  << " " << i << " " << Yp_to_Zw_conversion_table[i] << std::endl;
		
		Yp_pixels_times_two += Ypp_scale;
	}
	
#endif
}

