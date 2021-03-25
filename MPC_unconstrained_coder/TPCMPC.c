//This function requires dcc_con_base.h 
#include "Dcc_lib\dcc_con_base.h"
#include "TPCMPC.h" 

#include "Dcc_lib\dcc_Mat.h"
#include "Dcc_lib\dcc_Mat.c"

const double dVeMu_1x250[1][nNumPre] = {-34.6845804657639505,
-2.1349750864105022,
0.4758288998028854,
0.6764919400376537,
0.6832605793669867,
0.6745560620068205,
0.6647405545422719,
0.6549683558251733,
0.6453304679119084,
0.6358323180653076,
0.6264724717743830,
0.6172489693046941,
0.6081598354465808,
0.5992031201692757,
0.5903769015172265,
0.5816792854701838,
0.5731084055656853,
0.5646624225072855,
0.5563395237761707,
0.5481379232497763,
0.5400558608244505,
0.5320916020448697,
0.5242434377373166,
0.5165096836504512,
0.5088886800992282,
0.5013787916150867,
0.4939784066011750,
0.4866859369926161,
0.4794998179211911,
0.4724185073859332,
0.4654404859271715,
0.4585642563072415,
0.4517883431936509,
0.4451112928489362,
0.4385316728235275,
0.4320480716541890,
0.4256590985664202,
0.4193633831809899,
0.4131595752254696,
0.4070463442495886,
0.4010223793446251,
0.3950863888675005,
0.3892371001684987,
0.3834732593231089,
0.3777936308679450,
0.3721969975407146,
0.3666821600235624,
0.3612479366906834,
0.3558931633597380,
0.3506166930466116,
0.3454173957239917,
0.3402941580836720,
0.3352458833024912,
0.3302714908113591,
0.3253699160681248,
0.3205401103338520,
0.3157810404523279,
0.3110916886328336,
0.3064710522365343,
0.3019181435657012,
0.2974319896563649,
0.2930116320741423,
0.2886561267130554,
0.2843645435974826,
0.2801359666870208,
0.2759694936846998,
0.2718642358475439,
0.2678193178007487,
0.2638338773540698,
0.2599070653214972,
0.2560380453435940,
0.2522259937124884,
0.2484700991997157,
0.2447695628868062,
0.2411235979982340,
0.2375314297375013,
0.2339922951251205,
0.2305054428398468,
0.2270701330620486,
0.2236856373194776,
0.2203512383357877,
0.2170662298814707,
0.2138299166267878,
0.2106416139975526,
0.2075006480330793,
0.2044063552462281,
0.2013580824861224,
0.1983551868027267,
0.1953970353140585,
0.1924830050751606,
0.1896124829495975,
0.1867848654829144,
0.1839995587783089,
0.1812559783743257,
0.1785535491246388,
0.1758917050799700,
0.1732698893719144,
0.1706875540987764,
0.1681441602134899,
0.1656391774133482,
0.1631720840318699,
0.1607423669322779,
0.1583495214031548,
0.1559930510558341,
0.1536724677236017,
0.1513872913629397,
0.1491370499561806,
0.1469212794164386,
0.1447395234939269,
0.1425913336841990,
0.1404762691381810,
0.1383938965737261,
0.1363437901890633,
0.1343255315778278,
0.1323387096457940,
0.1303829205292532,
0.1284577675150358,
0.1265628609621028,
0.1246978182248366,
0.1228622635778465,
0.1210558281424262,
0.1192781498143765,
0.1175288731937791,
0.1158076495157751,
0.1141141365833610,
0.1124479987014002,
0.1108089066121649,
0.1091965374325224,
0.1076105745923944,
0.1060507077748553,
0.1045166328575139,
0.1030080518555241,
0.1015246728658807,
0.1000662100131934,
0.0986323833968695,
0.0972229190397589,
0.0958375488380322,
0.0944760105127056,
0.0931380475621895,
0.0918234092166569,
0.0905318503933193,
0.0892631316533486,
0.0880170191601423,
0.0867932846387089,
0.0855917053366840,
0.0844120639864292,
0.0832541487686277,
0.0821177532770529,
0.0810026764848474,
0.0799087227118762,
0.0788357015935528,
0.0777834280509077,
0.0767517222620132,
0.0757404096345560,
0.0747493207798530,
0.0737782914881033,
0.0728271627048997,
0.0718957805090601,
0.0709839960916886,
0.0700916657365986,
0.0692186508019396,
0.0683648177031143,
0.0675300378970314,
0.0667141878675233,
0.0659171491121780,
0.0651388081303143,
0.0643790564123464,
0.0636377904302992,
0.0629149116297849,
0.0622103264230483,
0.0615239461834806,
0.0608556872412574,
0.0602054708804212,
0.0595732233371437,
0.0589588757993014,
0.0583623644073344,
0.0577836302565168,
0.0572226194003749,
0.0566792828554676,
0.0561535766075691,
0.0556454616190591,
0.0551549038376611,
0.0546818742065220,
0.0542263486757161,
0.0537883082148695,
0.0533677388273848,
0.0529646315658738,
0.0525789825489707,
0.0522107929795369,
0.0518600691643068,
0.0515268225347552,
0.0512110696695887,
0.0509128323183993,
0.0506321374269576,
0.0503690171638174,
0.0501235089482701,
0.0498956554800676,
0.0496855047701290,
0.0494931101731583,
0.0493185304215186,
0.0491618296606050,
0.0490230774858131,
0.0489023489809476,
0.0487997247582303,
0.0487152909997866,
0.0486491395007678,
0.0486013677139879,
0.0485720787961900,
0.0485613816559145,
0.0485693910029884,
0.0485962273996486,
0.0486420173133041,
0.0487068931710615,
0.0487909934158002,
0.0488944625641000,
0.0490174512657807,
0.0491601163653009,
0.0493226209648165,
0.0495051344891005,
0.0497078327522581,
0.0499308980262386,
0.0501745191112766,
0.0504388914080306,
0.0507242169919342,
0.0510307046890820,
0.0513585701543504,
0.0517080359514018,
0.0520793316345535,
0.0524726938328941,
0.0528883663362441,
0.0533266001832628,
0.0537876537517184,
0.0542717928506996,
0.0547792908151908,
0.0553104286026463,
0.0558654948918909,
0.0564447861842352,
0.0570486069068067,
0.0576772695182784,
0.0583310946179018,
0.0590104110673126,
0.0597155562474544,
0.0604468779628005,
0.0612047567821975,
0.0619898821991699,
0.0628066815290434,
0.0637059098263286,
0.0653136700569785,
0.0754025231890884,
0.1905131330362786};

const double dMaMx_250x3[nNumPre][nStateNum] = {{0.6703200460356393,0.3296799539643608,0.0007032004603564},
{0.4493289641172216,0.5506710358827785,0.0024932896411722},
{0.3011942119122021,0.6988057880877980,0.0050119421191220},
{0.2018965179946554,0.7981034820053448,0.0080189651799466},
{0.1353352832366127,0.8646647167633875,0.0113533528323661},
{0.0907179532894125,0.9092820467105878,0.0149071795328941},
{0.0608100626252180,0.9391899373747823,0.0186081006262522},
{0.0407622039783662,0.9592377960216341,0.0224076220397837},
{0.0273237224472926,0.9726762775527077,0.0262732372244729},
{0.0183156388887342,0.9816843611112661,0.0301831563888873},
{0.0122773399030684,0.9877226600969319,0.0341227733990307},
{0.0082297470490200,0.9917702529509803,0.0380822974704902},
{0.0055165644207608,0.9944834355792396,0.0420551656442076},
{0.0036978637164829,0.9963021362835174,0.0460369786371648},
{0.0024787521766664,0.9975212478233340,0.0500247875217667},
{0.0016615572731739,0.9983384427268264,0.0540166155727318},
{0.0011137751478448,0.9988862248521555,0.0580111377514785},
{0.0007465858083767,0.9992534141916236,0.0620074658580838},
{0.0005004514334406,0.9994995485665596,0.0660050045143344},
{0.0003354626279025,0.9996645373720977,0.0700033546262791},
{0.0002248673241788,0.9997751326758214,0.0740022486732418},
{0.0001507330750955,0.9998492669249048,0.0780015073307510},
{0.0001010394018371,0.9998989605981632,0.0820010103940184},
{0.0000677287364909,0.9999322712635095,0.0860006772873649},
{0.0000453999297625,0.9999546000702378,0.0900004539992977},
{0.0000304324830084,0.9999695675169920,0.0940003043248301},
{0.0000203995034112,0.9999796004965892,0.0980002039950342},
{0.0000136741960657,0.9999863258039347,0.1020001367419607},
{0.0000091660877362,0.9999908339122641,0.1060000916608774},
{0.0000061442123533,0.9999938557876470,0.1100000614421236},
{0.0000041185887075,0.9999958814112928,0.1140000411858871},
{0.0000027607725720,0.9999972392274283,0.1180000276077258},
{0.0000018506011976,0.9999981493988028,0.1220000185060120},
{0.0000012404950800,0.9999987595049205,0.1260000124049508},
{0.0000008315287191,0.9999991684712813,0.1300000083152872},
{0.0000005573903693,0.9999994426096311,0.1340000055739037},
{0.0000003736299380,0.9999996263700625,0.1380000037362994},
{0.0000002504516372,0.9999997495483632,0.1420000025045164},
{0.0000001678827530,0.9999998321172475,0.1460000016788276},
{0.0000001125351747,0.9999998874648258,0.1500000011253518},
{0.0000000754345835,0.9999999245654171,0.1540000007543459},
{0.0000000505653135,0.9999999494346871,0.1580000005056532},
{0.0000000338949433,0.9999999661050573,0.1620000003389495},
{0.0000000227204599,0.9999999772795406,0.1660000002272046},
{0.0000000152299797,0.9999999847700208,0.1700000001522998},
{0.0000000102089607,0.9999999897910399,0.1740000001020896},
{0.0000000068432710,0.9999999931567296,0.1780000000684327},
{0.0000000045871817,0.9999999954128189,0.1820000000458718},
{0.0000000030748799,0.9999999969251208,0.1860000000307488},
{0.0000000020611536,0.9999999979388470,0.1900000000206116},
{0.0000000013816326,0.9999999986183680,0.1940000000138164},
{0.0000000009261360,0.9999999990738645,0.1980000000092614},
{0.0000000006208075,0.9999999993791929,0.2020000000062081},
{0.0000000004161397,0.9999999995838608,0.2060000000041615},
{0.0000000002789468,0.9999999997210537,0.2100000000027895},
{0.0000000001869836,0.9999999998130169,0.2140000000018699},
{0.0000000001253389,0.9999999998746617,0.2180000000012535},
{0.0000000000840172,0.9999999999159834,0.2220000000008402},
{0.0000000000563184,0.9999999999436822,0.2260000000005633},
{0.0000000000377513,0.9999999999622492,0.2300000000003776},
{0.0000000000253055,0.9999999999746950,0.2340000000002531},
{0.0000000000169628,0.9999999999830377,0.2380000000001697},
{0.0000000000113705,0.9999999999886300,0.2420000000001138},
{0.0000000000076219,0.9999999999923787,0.2460000000000763},
{0.0000000000051091,0.9999999999948914,0.2500000000000511},
{0.0000000000034247,0.9999999999965757,0.2540000000000343},
{0.0000000000022957,0.9999999999977048,0.2580000000000230},
{0.0000000000015388,0.9999999999984617,0.2620000000000155},
{0.0000000000010315,0.9999999999989690,0.2660000000000105},
{0.0000000000006914,0.9999999999993091,0.2700000000000071},
{0.0000000000004635,0.9999999999995370,0.2740000000000048},
{0.0000000000003107,0.9999999999996898,0.2780000000000032},
{0.0000000000002083,0.9999999999997923,0.2820000000000022},
{0.0000000000001396,0.9999999999998609,0.2860000000000016},
{0.0000000000000936,0.9999999999999070,0.2900000000000011},
{0.0000000000000627,0.9999999999999378,0.2940000000000008},
{0.0000000000000420,0.9999999999999585,0.2980000000000006},
{0.0000000000000282,0.9999999999999724,0.3020000000000004},
{0.0000000000000189,0.9999999999999817,0.3060000000000003},
{0.0000000000000127,0.9999999999999879,0.3100000000000003},
{0.0000000000000085,0.9999999999999921,0.3140000000000002},
{0.0000000000000057,0.9999999999999949,0.3180000000000002},
{0.0000000000000038,0.9999999999999968,0.3220000000000002},
{0.0000000000000026,0.9999999999999980,0.3260000000000002},
{0.0000000000000017,0.9999999999999989,0.3300000000000002},
{0.0000000000000011,0.9999999999999994,0.3340000000000002},
{0.0000000000000008,0.9999999999999998,0.3380000000000002},
{0.0000000000000005,1.0000000000000000,0.3420000000000002},
{0.0000000000000003,1.0000000000000002,0.3460000000000002},
{0.0000000000000002,1.0000000000000004,0.3500000000000002},
{0.0000000000000002,1.0000000000000004,0.3540000000000002},
{0.0000000000000001,1.0000000000000004,0.3580000000000002},
{0.0000000000000001,1.0000000000000004,0.3620000000000002},
{0.0000000000000000,1.0000000000000004,0.3660000000000002},
{0.0000000000000000,1.0000000000000004,0.3700000000000002},
{0.0000000000000000,1.0000000000000004,0.3740000000000002},
{0.0000000000000000,1.0000000000000004,0.3780000000000002},
{0.0000000000000000,1.0000000000000004,0.3820000000000002},
{0.0000000000000000,1.0000000000000004,0.3860000000000002},
{0.0000000000000000,1.0000000000000004,0.3900000000000002},
{0.0000000000000000,1.0000000000000004,0.3940000000000002},
{0.0000000000000000,1.0000000000000004,0.3980000000000002},
{0.0000000000000000,1.0000000000000004,0.4020000000000002},
{0.0000000000000000,1.0000000000000004,0.4060000000000002},
{0.0000000000000000,1.0000000000000004,0.4100000000000003},
{0.0000000000000000,1.0000000000000004,0.4140000000000003},
{0.0000000000000000,1.0000000000000004,0.4180000000000003},
{0.0000000000000000,1.0000000000000004,0.4220000000000003},
{0.0000000000000000,1.0000000000000004,0.4260000000000003},
{0.0000000000000000,1.0000000000000004,0.4300000000000003},
{0.0000000000000000,1.0000000000000004,0.4340000000000003},
{0.0000000000000000,1.0000000000000004,0.4380000000000003},
{0.0000000000000000,1.0000000000000004,0.4420000000000003},
{0.0000000000000000,1.0000000000000004,0.4460000000000003},
{0.0000000000000000,1.0000000000000004,0.4500000000000003},
{0.0000000000000000,1.0000000000000004,0.4540000000000003},
{0.0000000000000000,1.0000000000000004,0.4580000000000003},
{0.0000000000000000,1.0000000000000004,0.4620000000000003},
{0.0000000000000000,1.0000000000000004,0.4660000000000003},
{0.0000000000000000,1.0000000000000004,0.4700000000000003},
{0.0000000000000000,1.0000000000000004,0.4740000000000003},
{0.0000000000000000,1.0000000000000004,0.4780000000000003},
{0.0000000000000000,1.0000000000000004,0.4820000000000003},
{0.0000000000000000,1.0000000000000004,0.4860000000000003},
{0.0000000000000000,1.0000000000000004,0.4900000000000003},
{0.0000000000000000,1.0000000000000004,0.4940000000000003},
{0.0000000000000000,1.0000000000000004,0.4980000000000003},
{0.0000000000000000,1.0000000000000004,0.5020000000000003},
{0.0000000000000000,1.0000000000000004,0.5060000000000003},
{0.0000000000000000,1.0000000000000004,0.5100000000000003},
{0.0000000000000000,1.0000000000000004,0.5140000000000003},
{0.0000000000000000,1.0000000000000004,0.5180000000000003},
{0.0000000000000000,1.0000000000000004,0.5220000000000004},
{0.0000000000000000,1.0000000000000004,0.5260000000000004},
{0.0000000000000000,1.0000000000000004,0.5300000000000004},
{0.0000000000000000,1.0000000000000004,0.5340000000000004},
{0.0000000000000000,1.0000000000000004,0.5380000000000004},
{0.0000000000000000,1.0000000000000004,0.5420000000000004},
{0.0000000000000000,1.0000000000000004,0.5460000000000004},
{0.0000000000000000,1.0000000000000004,0.5500000000000004},
{0.0000000000000000,1.0000000000000004,0.5540000000000004},
{0.0000000000000000,1.0000000000000004,0.5580000000000004},
{0.0000000000000000,1.0000000000000004,0.5620000000000004},
{0.0000000000000000,1.0000000000000004,0.5660000000000004},
{0.0000000000000000,1.0000000000000004,0.5700000000000004},
{0.0000000000000000,1.0000000000000004,0.5740000000000004},
{0.0000000000000000,1.0000000000000004,0.5780000000000004},
{0.0000000000000000,1.0000000000000004,0.5820000000000004},
{0.0000000000000000,1.0000000000000004,0.5860000000000004},
{0.0000000000000000,1.0000000000000004,0.5900000000000004},
{0.0000000000000000,1.0000000000000004,0.5940000000000004},
{0.0000000000000000,1.0000000000000004,0.5980000000000004},
{0.0000000000000000,1.0000000000000004,0.6020000000000004},
{0.0000000000000000,1.0000000000000004,0.6060000000000004},
{0.0000000000000000,1.0000000000000004,0.6100000000000004},
{0.0000000000000000,1.0000000000000004,0.6140000000000004},
{0.0000000000000000,1.0000000000000004,0.6180000000000004},
{0.0000000000000000,1.0000000000000004,0.6220000000000004},
{0.0000000000000000,1.0000000000000004,0.6260000000000004},
{0.0000000000000000,1.0000000000000004,0.6300000000000004},
{0.0000000000000000,1.0000000000000004,0.6340000000000005},
{0.0000000000000000,1.0000000000000004,0.6380000000000005},
{0.0000000000000000,1.0000000000000004,0.6420000000000005},
{0.0000000000000000,1.0000000000000004,0.6460000000000005},
{0.0000000000000000,1.0000000000000004,0.6500000000000005},
{0.0000000000000000,1.0000000000000004,0.6540000000000005},
{0.0000000000000000,1.0000000000000004,0.6580000000000005},
{0.0000000000000000,1.0000000000000004,0.6620000000000005},
{0.0000000000000000,1.0000000000000004,0.6660000000000005},
{0.0000000000000000,1.0000000000000004,0.6700000000000005},
{0.0000000000000000,1.0000000000000004,0.6740000000000005},
{0.0000000000000000,1.0000000000000004,0.6780000000000005},
{0.0000000000000000,1.0000000000000004,0.6820000000000005},
{0.0000000000000000,1.0000000000000004,0.6860000000000005},
{0.0000000000000000,1.0000000000000004,0.6900000000000005},
{0.0000000000000000,1.0000000000000004,0.6940000000000005},
{0.0000000000000000,1.0000000000000004,0.6980000000000005},
{0.0000000000000000,1.0000000000000004,0.7020000000000005},
{0.0000000000000000,1.0000000000000004,0.7060000000000005},
{0.0000000000000000,1.0000000000000004,0.7100000000000005},
{0.0000000000000000,1.0000000000000004,0.7140000000000005},
{0.0000000000000000,1.0000000000000004,0.7180000000000005},
{0.0000000000000000,1.0000000000000004,0.7220000000000005},
{0.0000000000000000,1.0000000000000004,0.7260000000000005},
{0.0000000000000000,1.0000000000000004,0.7300000000000005},
{0.0000000000000000,1.0000000000000004,0.7340000000000005},
{0.0000000000000000,1.0000000000000004,0.7380000000000005},
{0.0000000000000000,1.0000000000000004,0.7420000000000005},
{0.0000000000000000,1.0000000000000004,0.7460000000000006},
{0.0000000000000000,1.0000000000000004,0.7500000000000006},
{0.0000000000000000,1.0000000000000004,0.7540000000000006},
{0.0000000000000000,1.0000000000000004,0.7580000000000006},
{0.0000000000000000,1.0000000000000004,0.7620000000000006},
{0.0000000000000000,1.0000000000000004,0.7660000000000006},
{0.0000000000000000,1.0000000000000004,0.7700000000000006},
{0.0000000000000000,1.0000000000000004,0.7740000000000006},
{0.0000000000000000,1.0000000000000004,0.7780000000000006},
{0.0000000000000000,1.0000000000000004,0.7820000000000006},
{0.0000000000000000,1.0000000000000004,0.7860000000000006},
{0.0000000000000000,1.0000000000000004,0.7900000000000006},
{0.0000000000000000,1.0000000000000004,0.7940000000000006},
{0.0000000000000000,1.0000000000000004,0.7980000000000006},
{0.0000000000000000,1.0000000000000004,0.8020000000000006},
{0.0000000000000000,1.0000000000000004,0.8060000000000006},
{0.0000000000000000,1.0000000000000004,0.8100000000000006},
{0.0000000000000000,1.0000000000000004,0.8140000000000006},
{0.0000000000000000,1.0000000000000004,0.8180000000000006},
{0.0000000000000000,1.0000000000000004,0.8220000000000006},
{0.0000000000000000,1.0000000000000004,0.8260000000000006},
{0.0000000000000000,1.0000000000000004,0.8300000000000006},
{0.0000000000000000,1.0000000000000004,0.8340000000000006},
{0.0000000000000000,1.0000000000000004,0.8380000000000006},
{0.0000000000000000,1.0000000000000004,0.8420000000000006},
{0.0000000000000000,1.0000000000000004,0.8460000000000006},
{0.0000000000000000,1.0000000000000004,0.8500000000000006},
{0.0000000000000000,1.0000000000000004,0.8540000000000006},
{0.0000000000000000,1.0000000000000004,0.8580000000000007},
{0.0000000000000000,1.0000000000000004,0.8620000000000007},
{0.0000000000000000,1.0000000000000004,0.8660000000000007},
{0.0000000000000000,1.0000000000000004,0.8700000000000007},
{0.0000000000000000,1.0000000000000004,0.8740000000000007},
{0.0000000000000000,1.0000000000000004,0.8780000000000007},
{0.0000000000000000,1.0000000000000004,0.8820000000000007},
{0.0000000000000000,1.0000000000000004,0.8860000000000007},
{0.0000000000000000,1.0000000000000004,0.8900000000000007},
{0.0000000000000000,1.0000000000000004,0.8940000000000007},
{0.0000000000000000,1.0000000000000004,0.8980000000000007},
{0.0000000000000000,1.0000000000000004,0.9020000000000007},
{0.0000000000000000,1.0000000000000004,0.9060000000000007},
{0.0000000000000000,1.0000000000000004,0.9100000000000007},
{0.0000000000000000,1.0000000000000004,0.9140000000000007},
{0.0000000000000000,1.0000000000000004,0.9180000000000007},
{0.0000000000000000,1.0000000000000004,0.9220000000000007},
{0.0000000000000000,1.0000000000000004,0.9260000000000007},
{0.0000000000000000,1.0000000000000004,0.9300000000000007},
{0.0000000000000000,1.0000000000000004,0.9340000000000007},
{0.0000000000000000,1.0000000000000004,0.9380000000000007},
{0.0000000000000000,1.0000000000000004,0.9420000000000007},
{0.0000000000000000,1.0000000000000004,0.9460000000000007},
{0.0000000000000000,1.0000000000000004,0.9500000000000007},
{0.0000000000000000,1.0000000000000004,0.9540000000000007},
{0.0000000000000000,1.0000000000000004,0.9580000000000007},
{0.0000000000000000,1.0000000000000004,0.9620000000000007},
{0.0000000000000000,1.0000000000000004,0.9660000000000007},
{0.0000000000000000,1.0000000000000004,0.9700000000000008},
{0.0000000000000000,1.0000000000000004,0.9740000000000008},
{0.0000000000000000,1.0000000000000004,0.9780000000000008},
{0.0000000000000000,1.0000000000000004,0.9820000000000008},
{0.0000000000000000,1.0000000000000004,0.9860000000000008},
{0.0000000000000000,1.0000000000000004,0.9860000000000008}};
const double dMPC_T = 0.004000;
double dTPCMPCConval[2];

/**
Ref trajectorys in x direction & y direction are required
To obtain a good control performance, States should be calcualted from an observer
To use as an tracking controller, States can calculate from Val^{sens} - Val^{ref}, and Ref trajectorys can be calculated from Tra^{ref} - Tra^{sens}, Tra^{sens} is expanded to the same dimension as Tra^{ref} but value is maintaining the same
*/
void fnvTPCMPCCalConval(double dVeZmpRefx_250x1[nNumPre][1], double dVeZmpRefy_250x1[nNumPre][1], double dVeStatex_3x1[nStateNum][1], double dVeStatey_3x1[nStateNum][1]) {
	double dVeZmpRelx_250x1[nNumPre][1], dVeZmpRely_250x1[nNumPre][1];
	dcc_fnvMatMet(&dMaMx_250x3[0][0], &dVeStatex_3x1[0][0], nNumPre, nStateNum, 1, '*', &dVeZmpRelx_250x1[0][0]);
	dcc_fnvMatMet(&dMaMx_250x3[0][0], &dVeStatey_3x1[0][0], nNumPre, nStateNum, 1, '*', &dVeZmpRely_250x1[0][0]);
	double dVeDeltaZmpx_250x1[nNumPre][1], dVeDeltaZmpy_250x1[nNumPre][1];
	dcc_fnvMatMet(&dVeZmpRefx_250x1[0][0], &dVeZmpRelx_250x1[0][0], nNumPre, 1, 1, '-', &dVeDeltaZmpx_250x1[0][0]);
	dcc_fnvMatMet(&dVeZmpRefy_250x1[0][0], &dVeZmpRely_250x1[0][0], nNumPre, 1, 1, '-', &dVeDeltaZmpy_250x1[0][0]);
	double dGaUx[1][1], dGaUy[1][1];
	dcc_fnvMatMet(&dVeMu_1x250[0][0], &dVeDeltaZmpx_250x1[0][0], 1, nNumPre, 1, '*', &dGaUx[0][0]);
	dcc_fnvMatMet(&dVeMu_1x250[0][0], &dVeDeltaZmpy_250x1[0][0], 1, nNumPre, 1, '*', &dGaUy[0][0]);

	dTPCMPCConval[0] = dGaUx[0][0];
	dTPCMPCConval[1] = dGaUy[0][0];

}

