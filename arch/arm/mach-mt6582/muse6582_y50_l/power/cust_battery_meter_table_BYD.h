#ifndef _CUST_BATTERY_METER_TABLE_BYD_H
#define _CUST_BATTERY_METER_TABLE_BYD_H

#define Q_MAX_POS_50_BYD		1838
#define Q_MAX_POS_25_BYD		1832
#define Q_MAX_POS_0_BYD			1588
#define Q_MAX_NEG_10_BYD		1483

#define Q_MAX_POS_50_H_CURRENT_BYD	1825
#define Q_MAX_POS_25_H_CURRENT_BYD	1808
#define Q_MAX_POS_0_H_CURRENT_BYD	1528
#define Q_MAX_NEG_10_H_CURRENT_BYD	949

/* Battery profile data BYD */

#define BATTERY_PROFILE_SIZE_BYD sizeof(battery_profile_t2_BYD) / sizeof(BATTERY_PROFILE_STRUC);
// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0_BYD[] =
{
	{0	,	4324},	// VC : 4324	0mA
	{2	,	4301},	// VC : 4087	30mA
	{4	,	4280},	// VC : 4056	60mA
	{6	,	4258},	// VC : 4028	90mA
	{8	,	4235},	// VC : 3998	119mA
	{10	,	4206},	// VC : 3961	149mA
	{12	,	4162},	// VC : 3893	179mA
	{14	,	4128},	// VC : 3816	209mA
	{16	,	4104},	// VC : 3770	239mA
	{18	,	4084},	// VC : 3740	269mA
	{20	,	4068},	// VC : 3717	298mA
	{22	,	4049},	// VC : 3695	328mA
	{24	,	4031},	// VC : 3670	358mA
	{26	,	4012},	// VC : 3647	388mA
	{28	,	3994},	// VC : 3625	418mA
	{30	,	3978},	// VC : 3607	448mA
	{32	,	3961},	// VC : 3591	478mA
	{34	,	3944},	// VC : 3575	507mA
	{36	,	3926},	// VC : 3559	537mA
	{38	,	3909},	// VC : 3546	567mA
	{40	,	3895},	// VC : 3531	597mA
	{42	,	3882},	// VC : 3517	627mA
	{44	,	3870},	// VC : 3503	657mA
	{46	,	3860},	// VC : 3493	686mA
	{48	,	3852},	// VC : 3480	716mA
	{50	,	3843},	// VC : 3468	746mA
	{52	,	3835},	// VC : 3457	776mA
	{54	,	3828},	// VC : 3447	806mA
	{56	,	3821},	// VC : 3436	836mA
	{58	,	3816},	// VC : 3427	866mA
	{60	,	3809},	// VC : 3417	895mA
	{62	,	3803},	// VC : 3408	925mA
	{64	,	3799},	// VC : 3398	955mA
	{66	,	3792},	// VC : 3390	985mA
	{68	,	3787},	// VC : 3381	1015mA
	{70	,	3781},	// VC : 3372	1045mA
	{72	,	3776},	// VC : 3363	1074mA
	{74	,	3769},	// VC : 3353	1104mA
	{76	,	3762},	// VC : 3342	1134mA
	{78	,	3755},	// VC : 3329	1164mA
	{81	,	3744},	// VC : 3315	1194mA
	{83	,	3733},	// VC : 3302	1224mA
	{85	,	3723},	// VC : 3287	1254mA
	{87	,	3714},	// VC : 3273	1283mA
	{89	,	3707},	// VC : 3258	1313mA
	{91	,	3704},	// VC : 3240	1343mA
	{93	,	3698},	// VC : 3213	1373mA
	{94	,	3694},	// VC : 3200	1394mA
	{95	,	3688},	// VC : 3200	1409mA
	{96	,	3683},	// VC : 3199	1421mA
	{96	,	3676},	// VC : 3200	1431mA
	{97	,	3669},	// VC : 3199	1439mA
	{98	,	3660},	// VC : 3198	1446mA
	{98	,	3654},	// VC : 3199	1451mA
	{98	,	3647},	// VC : 3200	1456mA
	{98	,	3640},	// VC : 3200	1460mA
	{99	,	3635},	// VC : 3200	1463mA
	{99	,	3628},	// VC : 3198	1466mA
	{99	,	3623},	// VC : 3199	1469mA
	{99	,	3618},	// VC : 3199	1471mA
	{99	,	3614},	// VC : 3198	1472mA
	{99	,	3610},	// VC : 3199	1474mA
	{99	,	3608},	// VC : 3199	1475mA
	{100	,	3605},	// VC : 3198	1476mA
	{100	,	3601},	// VC : 3199	1477mA
	{100	,	3598},	// VC : 3200	1478mA
	{100	,	3597},	// VC : 3199	1478mA
	{100	,	3595},	// VC : 3200	1479mA
	{100	,	3592},	// VC : 3196	1480mA
	{100	,	3590},	// VC : 3200	1480mA
	{100	,	3588},	// VC : 3196	1481mA
	{100	,	3587},	// VC : 3198	1482mA
	{100	,	3585},	// VC : 3195	1482mA
	{100	,	3582},	// VC : 3196	1483mA
	{100	,	3400},	// VC : 3197	1483mA
};

// T1 0C
BATTERY_PROFILE_STRUC battery_profile_t1_BYD[] =
{
	{0	,	4312},	// VC : 4312	0mA
	{2	,	4281},	// VC : 4133	30mA
	{4	,	4258},	// VC : 4107	60mA
	{6	,	4237},	// VC : 4084	90mA
	{7	,	4217},	// VC : 4061	119mA
	{9	,	4198},	// VC : 4041	149mA
	{11	,	4179},	// VC : 4019	179mA
	{13	,	4161},	// VC : 3997	209mA
	{15	,	4142},	// VC : 3977	239mA
	{17	,	4124},	// VC : 3956	269mA
	{19	,	4107},	// VC : 3936	298mA
	{21	,	4089},	// VC : 3916	328mA
	{23	,	4074},	// VC : 3896	358mA
	{24	,	4058},	// VC : 3878	388mA
	{26	,	4039},	// VC : 3859	418mA
	{28	,	4020},	// VC : 3840	448mA
	{30	,	4001},	// VC : 3822	478mA
	{32	,	3982},	// VC : 3805	507mA
	{34	,	3962},	// VC : 3791	537mA
	{36	,	3945},	// VC : 3777	567mA
	{38	,	3927},	// VC : 3764	597mA
	{39	,	3913},	// VC : 3750	627mA
	{41	,	3899},	// VC : 3738	657mA
	{43	,	3887},	// VC : 3726	686mA
	{45	,	3877},	// VC : 3715	716mA
	{47	,	3867},	// VC : 3703	746mA
	{49	,	3857},	// VC : 3694	776mA
	{51	,	3849},	// VC : 3684	806mA
	{53	,	3840},	// VC : 3674	836mA
	{55	,	3833},	// VC : 3665	866mA
	{56	,	3826},	// VC : 3657	895mA
	{58	,	3819},	// VC : 3648	925mA
	{60	,	3813},	// VC : 3640	955mA
	{62	,	3807},	// VC : 3632	985mA
	{64	,	3801},	// VC : 3625	1015mA
	{66	,	3796},	// VC : 3618	1045mA
	{68	,	3791},	// VC : 3610	1074mA
	{70	,	3786},	// VC : 3603	1104mA
	{71	,	3779},	// VC : 3596	1134mA
	{73	,	3773},	// VC : 3588	1164mA
	{75	,	3766},	// VC : 3580	1194mA
	{77	,	3758},	// VC : 3572	1224mA
	{79	,	3748},	// VC : 3562	1254mA
	{81	,	3737},	// VC : 3549	1283mA
	{83	,	3724},	// VC : 3535	1313mA
	{85	,	3712},	// VC : 3524	1343mA
	{86	,	3705},	// VC : 3517	1373mA
	{88	,	3701},	// VC : 3509	1403mA
	{90	,	3698},	// VC : 3502	1433mA
	{92	,	3695},	// VC : 3489	1462mA
	{94	,	3690},	// VC : 3464	1492mA
	{96	,	3661},	// VC : 3418	1522mA
	{98	,	3582},	// VC : 3334	1552mA
	{99	,	3461},	// VC : 3200	1578mA
	{100	,	3427},	// VC : 3198	1584mA
	{100	,	3409},	// VC : 3198	1586mA
	{100	,	3400},	// VC : 3200	1588mA
	{100	,	3392},	// VC : 3198	1589mA
	{100	,	3389},	// VC : 3199	1589mA
	{100	,	3385},	// VC : 3197	1590mA
	{100	,	3382},	// VC : 3199	1590mA
	{100	,	3380},	// VC : 3199	1590mA
	{100	,	3378},	// VC : 3197	1590mA
	{100	,	3377},	// VC : 3198	1590mA
	{100	,	3376},	// VC : 3195	1590mA
	{100	,	3373},	// VC : 3192	1590mA
	{100	,	3372},	// VC : 3193	1590mA
	{100	,	3371},	// VC : 3191	1590mA
	{100	,	3368},	// VC : 3187	1591mA
	{100	,	3367},	// VC : 3188	1591mA
	{100	,	3367},	// VC : 3187	1591mA
	{100	,	3363},	// VC : 3184	1591mA
	{100	,	3363},	// VC : 3183	1591mA
	{100	,	3361},	// VC : 3181	1591mA
	{100	,	3361},	// VC : 3196	1591mA
};

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2_BYD[] =
{
	{0	,	4335},	// VC : 4335	0mA
	{2	,	4309},	// VC : 4240	30mA
	{3	,	4286},	// VC : 4218	60mA
	{5	,	4267},	// VC : 4198	90mA
	{6	,	4246},	// VC : 4177	119mA
	{8	,	4227},	// VC : 4157	149mA
	{10	,	4208},	// VC : 4137	179mA
	{11	,	4189},	// VC : 4118	209mA
	{13	,	4171},	// VC : 4099	239mA
	{15	,	4152},	// VC : 4080	269mA
	{16	,	4135},	// VC : 4061	298mA
	{18	,	4117},	// VC : 4043	328mA
	{20	,	4100},	// VC : 4024	358mA
	{21	,	4083},	// VC : 4006	388mA
	{23	,	4067},	// VC : 3988	418mA
	{24	,	4051},	// VC : 3971	448mA
	{26	,	4036},	// VC : 3954	478mA
	{28	,	4020},	// VC : 3936	507mA
	{29	,	4006},	// VC : 3919	537mA
	{31	,	3992},	// VC : 3904	567mA
	{33	,	3980},	// VC : 3888	597mA
	{34	,	3967},	// VC : 3874	627mA
	{36	,	3954},	// VC : 3860	657mA
	{37	,	3942},	// VC : 3847	686mA
	{39	,	3930},	// VC : 3834	716mA
	{41	,	3917},	// VC : 3822	746mA
	{42	,	3901},	// VC : 3811	776mA
	{44	,	3884},	// VC : 3800	806mA
	{46	,	3868},	// VC : 3790	836mA
	{47	,	3855},	// VC : 3780	866mA
	{49	,	3844},	// VC : 3772	895mA
	{50	,	3835},	// VC : 3764	925mA
	{52	,	3827},	// VC : 3756	955mA
	{54	,	3820},	// VC : 3748	985mA
	{55	,	3813},	// VC : 3741	1015mA
	{57	,	3807},	// VC : 3735	1045mA
	{59	,	3801},	// VC : 3729	1074mA
	{60	,	3796},	// VC : 3724	1104mA
	{62	,	3791},	// VC : 3718	1134mA
	{64	,	3787},	// VC : 3714	1164mA
	{65	,	3783},	// VC : 3709	1194mA
	{67	,	3779},	// VC : 3705	1224mA
	{68	,	3777},	// VC : 3701	1254mA
	{70	,	3774},	// VC : 3698	1283mA
	{72	,	3772},	// VC : 3696	1313mA
	{73	,	3769},	// VC : 3694	1343mA
	{75	,	3768},	// VC : 3690	1373mA
	{77	,	3764},	// VC : 3687	1403mA
	{78	,	3759},	// VC : 3683	1433mA
	{80	,	3750},	// VC : 3677	1462mA
	{81	,	3744},	// VC : 3669	1492mA
	{83	,	3735},	// VC : 3661	1522mA
	{85	,	3721},	// VC : 3647	1552mA
	{86	,	3709},	// VC : 3634	1582mA
	{88	,	3692},	// VC : 3618	1612mA
	{90	,	3687},	// VC : 3614	1642mA
	{91	,	3685},	// VC : 3610	1671mA
	{93	,	3681},	// VC : 3604	1701mA
	{94	,	3676},	// VC : 3593	1731mA
	{96	,	3659},	// VC : 3568	1761mA
	{98	,	3589},	// VC : 3489	1791mA
	{99	,	3469},	// VC : 3338	1821mA
	{100	,	3362},	// VC : 3200	1838mA
	{100	,	3329},	// VC : 3198	1843mA
	{100	,	3312},	// VC : 3198	1846mA
	{100	,	3302},	// VC : 3197	1847mA
	{100	,	3298},	// VC : 3198	1848mA
	{100	,	3293},	// VC : 3197	1849mA
	{100	,	3290},	// VC : 3199	1850mA
	{100	,	3287},	// VC : 3199	1850mA
	{100	,	3284},	// VC : 3196	1851mA
	{100	,	3283},	// VC : 3199	1851mA
	{100	,	3280},	// VC : 3198	1851mA
	{100	,	3279},	// VC : 3195	1851mA
	{100	,	3279},	// VC : 3198	1851mA
};

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3_BYD[] =
{
	{0	,	4342},	// VC : 4342	0mA
	{2	,	4316},	// VC : 4262	30mA
	{3	,	4294},	// VC : 4239	60mA
	{5	,	4273},	// VC : 4218	90mA
	{6	,	4254},	// VC : 4197	119mA
	{8	,	4233},	// VC : 4177	149mA
	{10	,	4213},	// VC : 4157	179mA
	{11	,	4194},	// VC : 4139	209mA
	{13	,	4176},	// VC : 4119	239mA
	{15	,	4157},	// VC : 4100	269mA
	{16	,	4139},	// VC : 4082	298mA
	{18	,	4121},	// VC : 4063	328mA
	{19	,	4103},	// VC : 4045	358mA
	{21	,	4087},	// VC : 4027	388mA
	{23	,	4070},	// VC : 4011	418mA
	{24	,	4054},	// VC : 3994	448mA
	{26	,	4038},	// VC : 3978	478mA
	{28	,	4023},	// VC : 3961	507mA
	{29	,	4007},	// VC : 3946	537mA
	{31	,	3994},	// VC : 3931	567mA
	{32	,	3980},	// VC : 3917	597mA
	{34	,	3967},	// VC : 3902	627mA
	{36	,	3954},	// VC : 3889	657mA
	{37	,	3942},	// VC : 3874	686mA
	{39	,	3930},	// VC : 3860	716mA
	{41	,	3919},	// VC : 3847	746mA
	{42	,	3907},	// VC : 3833	776mA
	{44	,	3895},	// VC : 3820	806mA
	{45	,	3880},	// VC : 3808	836mA
	{47	,	3862},	// VC : 3797	866mA
	{49	,	3848},	// VC : 3787	895mA
	{50	,	3837},	// VC : 3778	925mA
	{52	,	3828},	// VC : 3769	955mA
	{54	,	3820},	// VC : 3762	985mA
	{55	,	3813},	// VC : 3754	1015mA
	{57	,	3806},	// VC : 3748	1045mA
	{58	,	3800},	// VC : 3741	1074mA
	{60	,	3795},	// VC : 3735	1104mA
	{62	,	3789},	// VC : 3730	1134mA
	{63	,	3784},	// VC : 3725	1164mA
	{65	,	3781},	// VC : 3720	1194mA
	{67	,	3776},	// VC : 3715	1224mA
	{68	,	3773},	// VC : 3711	1254mA
	{70	,	3770},	// VC : 3706	1283mA
	{71	,	3766},	// VC : 3702	1313mA
	{73	,	3759},	// VC : 3698	1343mA
	{75	,	3749},	// VC : 3691	1373mA
	{76	,	3745},	// VC : 3686	1403mA
	{78	,	3739},	// VC : 3681	1433mA
	{80	,	3733},	// VC : 3675	1462mA
	{81	,	3726},	// VC : 3667	1492mA
	{83	,	3719},	// VC : 3662	1522mA
	{84	,	3709},	// VC : 3651	1552mA
	{86	,	3695},	// VC : 3637	1582mA
	{88	,	3680},	// VC : 3622	1612mA
	{89	,	3669},	// VC : 3612	1642mA
	{91	,	3667},	// VC : 3609	1671mA
	{93	,	3663},	// VC : 3604	1701mA
	{94	,	3659},	// VC : 3596	1731mA
	{96	,	3648},	// VC : 3584	1761mA
	{97	,	3598},	// VC : 3531	1791mA
	{100	,	3500},	// VC : 3429	1821mA
	{100	,	3330},	// VC : 3248	1850mA
	{100	,	3270},	// VC : 3199	1857mA
	{100	,	3260},	// VC : 3199	1858mA
	{100	,	3256},	// VC : 3200	1859mA
	{100	,	3254},	// VC : 3198	1859mA
	{100	,	3253},	// VC : 3197	1859mA
	{100	,	3252},	// VC : 3197	1859mA
	{100	,	3251},	// VC : 3199	1859mA
	{100	,	3251},	// VC : 3196	1859mA
	{100	,	3251},	// VC : 3195	1859mA
	{100	,	3250},	// VC : 3195	1859mA
	{100	,	3248},	// VC : 3194	1859mA
	{100	,	3248},	// VC : 3194	1859mA
};

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature_BYD[] =
{
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
};

#define R_PROFILE_SIZE_BYD sizeof(r_profile_t2_BYD) / sizeof(R_PROFILE_STRUC);
// ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================
// T0 -10C
R_PROFILE_STRUC r_profile_t0_BYD[] =
{
	{275	,	4324},		// VC : 4324	0mA
	{535	,	4301},		// VC : 4087	30mA
	{560	,	4280},		// VC : 4056	60mA
	{575	,	4258},		// VC : 4028	90mA
	{593	,	4235},		// VC : 3998	119mA
	{613	,	4206},		// VC : 3961	149mA
	{673	,	4162},		// VC : 3893	179mA
	{780	,	4128},		// VC : 3816	209mA
	{835	,	4104},		// VC : 3770	239mA
	{860	,	4084},		// VC : 3740	269mA
	{878	,	4068},		// VC : 3717	298mA
	{885	,	4049},		// VC : 3695	328mA
	{903	,	4031},		// VC : 3670	358mA
	{913	,	4012},		// VC : 3647	388mA
	{923	,	3994},		// VC : 3625	418mA
	{928	,	3978},		// VC : 3607	448mA
	{925	,	3961},		// VC : 3591	478mA
	{923	,	3944},		// VC : 3575	507mA
	{918	,	3926},		// VC : 3559	537mA
	{908	,	3909},		// VC : 3546	567mA
	{910	,	3895},		// VC : 3531	597mA
	{913	,	3882},		// VC : 3517	627mA
	{918	,	3870},		// VC : 3503	657mA
	{918	,	3860},		// VC : 3493	686mA
	{930	,	3852},		// VC : 3480	716mA
	{938	,	3843},		// VC : 3468	746mA
	{945	,	3835},		// VC : 3457	776mA
	{953	,	3828},		// VC : 3447	806mA
	{963	,	3821},		// VC : 3436	836mA
	{973	,	3816},		// VC : 3427	866mA
	{980	,	3809},		// VC : 3417	895mA
	{988	,	3803},		// VC : 3408	925mA
	{1003	,	3799},		// VC : 3398	955mA
	{1005	,	3792},		// VC : 3390	985mA
	{1015	,	3787},		// VC : 3381	1015mA
	{1023	,	3781},		// VC : 3372	1045mA
	{1033	,	3776},		// VC : 3363	1074mA
	{1040	,	3769},		// VC : 3353	1104mA
	{1050	,	3762},		// VC : 3342	1134mA
	{1065	,	3755},		// VC : 3329	1164mA
	{1073	,	3744},		// VC : 3315	1194mA
	{1078	,	3733},		// VC : 3302	1224mA
	{1090	,	3723},		// VC : 3287	1254mA
	{1103	,	3714},		// VC : 3273	1283mA
	{1123	,	3707},		// VC : 3258	1313mA
	{1160	,	3704},		// VC : 3240	1343mA
	{1213	,	3698},		// VC : 3213	1373mA
	{1235	,	3694},		// VC : 3200	1394mA
	{1220	,	3688},		// VC : 3200	1409mA
	{1210	,	3683},		// VC : 3199	1421mA
	{1190	,	3676},		// VC : 3200	1431mA
	{1175	,	3669},		// VC : 3199	1439mA
	{1155	,	3660},		// VC : 3198	1446mA
	{1138	,	3654},		// VC : 3199	1451mA
	{1118	,	3647},		// VC : 3200	1456mA
	{1100	,	3640},		// VC : 3200	1460mA
	{1088	,	3635},		// VC : 3200	1463mA
	{1075	,	3628},		// VC : 3198	1466mA
	{1060	,	3623},		// VC : 3199	1469mA
	{1048	,	3618},		// VC : 3199	1471mA
	{1040	,	3614},		// VC : 3198	1472mA
	{1028	,	3610},		// VC : 3199	1474mA
	{1023	,	3608},		// VC : 3199	1475mA
	{1018	,	3605},		// VC : 3198	1476mA
	{1005	,	3601},		// VC : 3199	1477mA
	{995	,	3598},		// VC : 3200	1478mA
	{995	,	3597},		// VC : 3199	1478mA
	{988	,	3595},		// VC : 3200	1479mA
	{990	,	3592},		// VC : 3196	1480mA
	{975	,	3590},		// VC : 3200	1480mA
	{980	,	3588},		// VC : 3196	1481mA
	{973	,	3587},		// VC : 3198	1482mA
	{975	,	3585},		// VC : 3195	1482mA
	{965	,	3582},		// VC : 3196	1483mA
	{508	,	3400},		// VC : 3197	1483mA
};

// T1 0C
R_PROFILE_STRUC r_profile_t1_BYD[] =
{
	{525	,	4312},		// VC : 4312	0mA
	{370	,	4281},		// VC : 4133	30mA
	{378	,	4258},		// VC : 4107	60mA
	{383	,	4237},		// VC : 4084	90mA
	{390	,	4217},		// VC : 4061	119mA
	{393	,	4198},		// VC : 4041	149mA
	{400	,	4179},		// VC : 4019	179mA
	{410	,	4161},		// VC : 3997	209mA
	{413	,	4142},		// VC : 3977	239mA
	{420	,	4124},		// VC : 3956	269mA
	{428	,	4107},		// VC : 3936	298mA
	{433	,	4089},		// VC : 3916	328mA
	{445	,	4074},		// VC : 3896	358mA
	{450	,	4058},		// VC : 3878	388mA
	{450	,	4039},		// VC : 3859	418mA
	{450	,	4020},		// VC : 3840	448mA
	{448	,	4001},		// VC : 3822	478mA
	{443	,	3982},		// VC : 3805	507mA
	{428	,	3962},		// VC : 3791	537mA
	{420	,	3945},		// VC : 3777	567mA
	{408	,	3927},		// VC : 3764	597mA
	{408	,	3913},		// VC : 3750	627mA
	{403	,	3899},		// VC : 3738	657mA
	{403	,	3887},		// VC : 3726	686mA
	{405	,	3877},		// VC : 3715	716mA
	{410	,	3867},		// VC : 3703	746mA
	{408	,	3857},		// VC : 3694	776mA
	{413	,	3849},		// VC : 3684	806mA
	{415	,	3840},		// VC : 3674	836mA
	{420	,	3833},		// VC : 3665	866mA
	{423	,	3826},		// VC : 3657	895mA
	{428	,	3819},		// VC : 3648	925mA
	{433	,	3813},		// VC : 3640	955mA
	{438	,	3807},		// VC : 3632	985mA
	{440	,	3801},		// VC : 3625	1015mA
	{445	,	3796},		// VC : 3618	1045mA
	{453	,	3791},		// VC : 3610	1074mA
	{458	,	3786},		// VC : 3603	1104mA
	{458	,	3779},		// VC : 3596	1134mA
	{463	,	3773},		// VC : 3588	1164mA
	{465	,	3766},		// VC : 3580	1194mA
	{465	,	3758},		// VC : 3572	1224mA
	{465	,	3748},		// VC : 3562	1254mA
	{470	,	3737},		// VC : 3549	1283mA
	{473	,	3724},		// VC : 3535	1313mA
	{470	,	3712},		// VC : 3524	1343mA
	{470	,	3705},		// VC : 3517	1373mA
	{480	,	3701},		// VC : 3509	1403mA
	{490	,	3698},		// VC : 3502	1433mA
	{515	,	3695},		// VC : 3489	1462mA
	{565	,	3690},		// VC : 3464	1492mA
	{608	,	3661},		// VC : 3418	1522mA
	{620	,	3582},		// VC : 3334	1552mA
	{653	,	3461},		// VC : 3200	1578mA
	{573	,	3427},		// VC : 3198	1584mA
	{528	,	3409},		// VC : 3198	1586mA
	{500	,	3400},		// VC : 3200	1588mA
	{485	,	3392},		// VC : 3198	1589mA
	{475	,	3389},		// VC : 3199	1589mA
	{470	,	3385},		// VC : 3197	1590mA
	{458	,	3382},		// VC : 3199	1590mA
	{453	,	3380},		// VC : 3199	1590mA
	{453	,	3378},		// VC : 3197	1590mA
	{448	,	3377},		// VC : 3198	1590mA
	{453	,	3376},		// VC : 3195	1590mA
	{453	,	3373},		// VC : 3192	1590mA
	{448	,	3372},		// VC : 3193	1590mA
	{450	,	3371},		// VC : 3191	1590mA
	{453	,	3368},		// VC : 3187	1591mA
	{448	,	3367},		// VC : 3188	1591mA
	{450	,	3367},		// VC : 3187	1591mA
	{448	,	3363},		// VC : 3184	1591mA
	{450	,	3363},		// VC : 3183	1591mA
	{450	,	3361},		// VC : 3181	1591mA
	{413	,	3361},		// VC : 3196	1591mA
};

// T2 25C
R_PROFILE_STRUC r_profile_t2_BYD[] =
{
	{233	,	4335},		// VC : 4335	0mA
	{173	,	4309},		// VC : 4240	30mA
	{170	,	4286},		// VC : 4218	60mA
	{173	,	4267},		// VC : 4198	90mA
	{173	,	4246},		// VC : 4177	119mA
	{175	,	4227},		// VC : 4157	149mA
	{178	,	4208},		// VC : 4137	179mA
	{178	,	4189},		// VC : 4118	209mA
	{180	,	4171},		// VC : 4099	239mA
	{180	,	4152},		// VC : 4080	269mA
	{185	,	4135},		// VC : 4061	298mA
	{185	,	4117},		// VC : 4043	328mA
	{190	,	4100},		// VC : 4024	358mA
	{193	,	4083},		// VC : 4006	388mA
	{198	,	4067},		// VC : 3988	418mA
	{200	,	4051},		// VC : 3971	448mA
	{205	,	4036},		// VC : 3954	478mA
	{210	,	4020},		// VC : 3936	507mA
	{218	,	4006},		// VC : 3919	537mA
	{220	,	3992},		// VC : 3904	567mA
	{230	,	3980},		// VC : 3888	597mA
	{233	,	3967},		// VC : 3874	627mA
	{235	,	3954},		// VC : 3860	657mA
	{238	,	3942},		// VC : 3847	686mA
	{240	,	3930},		// VC : 3834	716mA
	{238	,	3917},		// VC : 3822	746mA
	{225	,	3901},		// VC : 3811	776mA
	{210	,	3884},		// VC : 3800	806mA
	{195	,	3868},		// VC : 3790	836mA
	{188	,	3855},		// VC : 3780	866mA
	{180	,	3844},		// VC : 3772	895mA
	{178	,	3835},		// VC : 3764	925mA
	{178	,	3827},		// VC : 3756	955mA
	{180	,	3820},		// VC : 3748	985mA
	{180	,	3813},		// VC : 3741	1015mA
	{180	,	3807},		// VC : 3735	1045mA
	{180	,	3801},		// VC : 3729	1074mA
	{180	,	3796},		// VC : 3724	1104mA
	{183	,	3791},		// VC : 3718	1134mA
	{183	,	3787},		// VC : 3714	1164mA
	{185	,	3783},		// VC : 3709	1194mA
	{185	,	3779},		// VC : 3705	1224mA
	{190	,	3777},		// VC : 3701	1254mA
	{190	,	3774},		// VC : 3698	1283mA
	{190	,	3772},		// VC : 3696	1313mA
	{188	,	3769},		// VC : 3694	1343mA
	{195	,	3768},		// VC : 3690	1373mA
	{193	,	3764},		// VC : 3687	1403mA
	{190	,	3759},		// VC : 3683	1433mA
	{183	,	3750},		// VC : 3677	1462mA
	{188	,	3744},		// VC : 3669	1492mA
	{185	,	3735},		// VC : 3661	1522mA
	{185	,	3721},		// VC : 3647	1552mA
	{188	,	3709},		// VC : 3634	1582mA
	{185	,	3692},		// VC : 3618	1612mA
	{183	,	3687},		// VC : 3614	1642mA
	{188	,	3685},		// VC : 3610	1671mA
	{193	,	3681},		// VC : 3604	1701mA
	{208	,	3676},		// VC : 3593	1731mA
	{228	,	3659},		// VC : 3568	1761mA
	{250	,	3589},		// VC : 3489	1791mA
	{328	,	3469},		// VC : 3338	1821mA
	{405	,	3362},		// VC : 3200	1838mA
	{328	,	3329},		// VC : 3198	1843mA
	{285	,	3312},		// VC : 3198	1846mA
	{263	,	3302},		// VC : 3197	1847mA
	{250	,	3298},		// VC : 3198	1848mA
	{240	,	3293},		// VC : 3197	1849mA
	{228	,	3290},		// VC : 3199	1850mA
	{220	,	3287},		// VC : 3199	1850mA
	{220	,	3284},		// VC : 3196	1851mA
	{210	,	3283},		// VC : 3199	1851mA
	{205	,	3280},		// VC : 3198	1851mA
	{210	,	3279},		// VC : 3195	1851mA
	{203	,	3279},		// VC : 3198	1851mA
};

// T3 50C
R_PROFILE_STRUC r_profile_t3_BYD[] =
{
	{105	,	4342},		// VC : 4342	0mA
	{135	,	4316},		// VC : 4262	30mA
	{138	,	4294},		// VC : 4239	60mA
	{138	,	4273},		// VC : 4218	90mA
	{143	,	4254},		// VC : 4197	119mA
	{140	,	4233},		// VC : 4177	149mA
	{140	,	4213},		// VC : 4157	179mA
	{138	,	4194},		// VC : 4139	209mA
	{143	,	4176},		// VC : 4119	239mA
	{143	,	4157},		// VC : 4100	269mA
	{143	,	4139},		// VC : 4082	298mA
	{145	,	4121},		// VC : 4063	328mA
	{145	,	4103},		// VC : 4045	358mA
	{150	,	4087},		// VC : 4027	388mA
	{148	,	4070},		// VC : 4011	418mA
	{150	,	4054},		// VC : 3994	448mA
	{150	,	4038},		// VC : 3978	478mA
	{155	,	4023},		// VC : 3961	507mA
	{153	,	4007},		// VC : 3946	537mA
	{158	,	3994},		// VC : 3931	567mA
	{158	,	3980},		// VC : 3917	597mA
	{163	,	3967},		// VC : 3902	627mA
	{163	,	3954},		// VC : 3889	657mA
	{170	,	3942},		// VC : 3874	686mA
	{175	,	3930},		// VC : 3860	716mA
	{180	,	3919},		// VC : 3847	746mA
	{185	,	3907},		// VC : 3833	776mA
	{188	,	3895},		// VC : 3820	806mA
	{180	,	3880},		// VC : 3808	836mA
	{163	,	3862},		// VC : 3797	866mA
	{153	,	3848},		// VC : 3787	895mA
	{148	,	3837},		// VC : 3778	925mA
	{148	,	3828},		// VC : 3769	955mA
	{145	,	3820},		// VC : 3762	985mA
	{148	,	3813},		// VC : 3754	1015mA
	{145	,	3806},		// VC : 3748	1045mA
	{148	,	3800},		// VC : 3741	1074mA
	{150	,	3795},		// VC : 3735	1104mA
	{148	,	3789},		// VC : 3730	1134mA
	{148	,	3784},		// VC : 3725	1164mA
	{153	,	3781},		// VC : 3720	1194mA
	{153	,	3776},		// VC : 3715	1224mA
	{155	,	3773},		// VC : 3711	1254mA
	{160	,	3770},		// VC : 3706	1283mA
	{160	,	3766},		// VC : 3702	1313mA
	{153	,	3759},		// VC : 3698	1343mA
	{145	,	3749},		// VC : 3691	1373mA
	{148	,	3745},		// VC : 3686	1403mA
	{145	,	3739},		// VC : 3681	1433mA
	{145	,	3733},		// VC : 3675	1462mA
	{148	,	3726},		// VC : 3667	1492mA
	{143	,	3719},		// VC : 3662	1522mA
	{145	,	3709},		// VC : 3651	1552mA
	{145	,	3695},		// VC : 3637	1582mA
	{145	,	3680},		// VC : 3622	1612mA
	{143	,	3669},		// VC : 3612	1642mA
	{145	,	3667},		// VC : 3609	1671mA
	{148	,	3663},		// VC : 3604	1701mA
	{158	,	3659},		// VC : 3596	1731mA
	{160	,	3648},		// VC : 3584	1761mA
	{168	,	3598},		// VC : 3531	1791mA
	{178	,	3500},		// VC : 3429	1821mA
	{205	,	3330},		// VC : 3248	1850mA
	{178	,	3270},		// VC : 3199	1857mA
	{153	,	3260},		// VC : 3199	1858mA
	{140	,	3256},		// VC : 3200	1859mA
	{140	,	3254},		// VC : 3198	1859mA
	{140	,	3253},		// VC : 3197	1859mA
	{138	,	3252},		// VC : 3197	1859mA
	{130	,	3251},		// VC : 3199	1859mA
	{138	,	3251},		// VC : 3196	1859mA
	{140	,	3251},		// VC : 3195	1859mA
	{138	,	3250},		// VC : 3195	1859mA
	{135	,	3248},		// VC : 3194	1859mA
	{135	,	3248},		// VC : 3194	1859mA
};

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature_BYD[] =
{
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
};

#endif /* _CUST_BATTERY_METER_TABLE_BYD_H */