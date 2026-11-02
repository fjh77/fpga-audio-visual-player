// =====================================================================
// curl_lut.vh —— 页面卷曲转场的全部常数表（自动生成，勿手改）
// 生成器：tools/curl_sim/gen_curl_hw.py
// 风格：arc（rmax=22.0 k=6.0 lift_frac=1.95）
// 帧数：70 档（59.5238Hz 下 1.176 秒）
//
// ★ 这张表同时被 RTL 与 Python 定点模型 gen_curl_hw.hw_render 使用 ★
// ★ 禁止在别处重写这里的常数；要改风格必须重新跑生成器 ★
// =====================================================================

// sin(pi*(x+0.5)/W) * 16384，Q14。RTL 按 x<320 ? x : 639-x 取镜像
// 长度 320，值域 15 位
function [14:0] curl_sin;
    input [8:0] idx;
    begin
        case (idx)
        0: curl_sin = 15'd40;  1: curl_sin = 15'd121;  2: curl_sin = 15'd201;  3: curl_sin = 15'd281;  4: curl_sin = 15'd362;  5: curl_sin = 15'd442;  6: curl_sin = 15'd523;  7: curl_sin = 15'd603;
        8: curl_sin = 15'd683;  9: curl_sin = 15'd764;  10: curl_sin = 15'd844;  11: curl_sin = 15'd924;  12: curl_sin = 15'd1005;  13: curl_sin = 15'd1085;  14: curl_sin = 15'd1165;  15: curl_sin = 15'd1245;
        16: curl_sin = 15'd1326;  17: curl_sin = 15'd1406;  18: curl_sin = 15'd1486;  19: curl_sin = 15'd1566;  20: curl_sin = 15'd1646;  21: curl_sin = 15'd1726;  22: curl_sin = 15'd1806;  23: curl_sin = 15'd1886;
        24: curl_sin = 15'd1966;  25: curl_sin = 15'd2045;  26: curl_sin = 15'd2125;  27: curl_sin = 15'd2205;  28: curl_sin = 15'd2285;  29: curl_sin = 15'd2364;  30: curl_sin = 15'd2444;  31: curl_sin = 15'd2523;
        32: curl_sin = 15'd2603;  33: curl_sin = 15'd2682;  34: curl_sin = 15'd2761;  35: curl_sin = 15'd2841;  36: curl_sin = 15'd2920;  37: curl_sin = 15'd2999;  38: curl_sin = 15'd3078;  39: curl_sin = 15'd3157;
        40: curl_sin = 15'd3236;  41: curl_sin = 15'd3315;  42: curl_sin = 15'd3393;  43: curl_sin = 15'd3472;  44: curl_sin = 15'd3551;  45: curl_sin = 15'd3629;  46: curl_sin = 15'd3707;  47: curl_sin = 15'd3786;
        48: curl_sin = 15'd3864;  49: curl_sin = 15'd3942;  50: curl_sin = 15'd4020;  51: curl_sin = 15'd4098;  52: curl_sin = 15'd4176;  53: curl_sin = 15'd4253;  54: curl_sin = 15'd4331;  55: curl_sin = 15'd4409;
        56: curl_sin = 15'd4486;  57: curl_sin = 15'd4563;  58: curl_sin = 15'd4640;  59: curl_sin = 15'd4718;  60: curl_sin = 15'd4794;  61: curl_sin = 15'd4871;  62: curl_sin = 15'd4948;  63: curl_sin = 15'd5025;
        64: curl_sin = 15'd5101;  65: curl_sin = 15'd5178;  66: curl_sin = 15'd5254;  67: curl_sin = 15'd5330;  68: curl_sin = 15'd5406;  69: curl_sin = 15'd5482;  70: curl_sin = 15'd5557;  71: curl_sin = 15'd5633;
        72: curl_sin = 15'd5708;  73: curl_sin = 15'd5784;  74: curl_sin = 15'd5859;  75: curl_sin = 15'd5934;  76: curl_sin = 15'd6009;  77: curl_sin = 15'd6084;  78: curl_sin = 15'd6158;  79: curl_sin = 15'd6233;
        80: curl_sin = 15'd6307;  81: curl_sin = 15'd6381;  82: curl_sin = 15'd6455;  83: curl_sin = 15'd6529;  84: curl_sin = 15'd6603;  85: curl_sin = 15'd6676;  86: curl_sin = 15'd6750;  87: curl_sin = 15'd6823;
        88: curl_sin = 15'd6896;  89: curl_sin = 15'd6969;  90: curl_sin = 15'd7041;  91: curl_sin = 15'd7114;  92: curl_sin = 15'd7186;  93: curl_sin = 15'd7258;  94: curl_sin = 15'd7330;  95: curl_sin = 15'd7402;
        96: curl_sin = 15'd7474;  97: curl_sin = 15'd7545;  98: curl_sin = 15'd7617;  99: curl_sin = 15'd7688;  100: curl_sin = 15'd7759;  101: curl_sin = 15'd7830;  102: curl_sin = 15'd7900;  103: curl_sin = 15'd7970;
        104: curl_sin = 15'd8041;  105: curl_sin = 15'd8111;  106: curl_sin = 15'd8180;  107: curl_sin = 15'd8250;  108: curl_sin = 15'd8319;  109: curl_sin = 15'd8389;  110: curl_sin = 15'd8458;  111: curl_sin = 15'd8526;
        112: curl_sin = 15'd8595;  113: curl_sin = 15'd8663;  114: curl_sin = 15'd8731;  115: curl_sin = 15'd8799;  116: curl_sin = 15'd8867;  117: curl_sin = 15'd8935;  118: curl_sin = 15'd9002;  119: curl_sin = 15'd9069;
        120: curl_sin = 15'd9136;  121: curl_sin = 15'd9203;  122: curl_sin = 15'd9269;  123: curl_sin = 15'd9335;  124: curl_sin = 15'd9401;  125: curl_sin = 15'd9467;  126: curl_sin = 15'd9532;  127: curl_sin = 15'd9598;
        128: curl_sin = 15'd9663;  129: curl_sin = 15'd9728;  130: curl_sin = 15'd9792;  131: curl_sin = 15'd9857;  132: curl_sin = 15'd9921;  133: curl_sin = 15'd9985;  134: curl_sin = 15'd10048;  135: curl_sin = 15'd10112;
        136: curl_sin = 15'd10175;  137: curl_sin = 15'd10238;  138: curl_sin = 15'd10300;  139: curl_sin = 15'd10363;  140: curl_sin = 15'd10425;  141: curl_sin = 15'd10487;  142: curl_sin = 15'd10549;  143: curl_sin = 15'd10610;
        144: curl_sin = 15'd10671;  145: curl_sin = 15'd10732;  146: curl_sin = 15'd10793;  147: curl_sin = 15'd10853;  148: curl_sin = 15'd10913;  149: curl_sin = 15'd10973;  150: curl_sin = 15'd11033;  151: curl_sin = 15'd11092;
        152: curl_sin = 15'd11151;  153: curl_sin = 15'd11210;  154: curl_sin = 15'd11268;  155: curl_sin = 15'd11327;  156: curl_sin = 15'd11384;  157: curl_sin = 15'd11442;  158: curl_sin = 15'd11500;  159: curl_sin = 15'd11557;
        160: curl_sin = 15'd11614;  161: curl_sin = 15'd11670;  162: curl_sin = 15'd11727;  163: curl_sin = 15'd11783;  164: curl_sin = 15'd11838;  165: curl_sin = 15'd11894;  166: curl_sin = 15'd11949;  167: curl_sin = 15'd12004;
        168: curl_sin = 15'd12058;  169: curl_sin = 15'd12113;  170: curl_sin = 15'd12167;  171: curl_sin = 15'd12220;  172: curl_sin = 15'd12274;  173: curl_sin = 15'd12327;  174: curl_sin = 15'd12380;  175: curl_sin = 15'd12432;
        176: curl_sin = 15'd12485;  177: curl_sin = 15'd12537;  178: curl_sin = 15'd12588;  179: curl_sin = 15'd12639;  180: curl_sin = 15'd12690;  181: curl_sin = 15'd12741;  182: curl_sin = 15'd12792;  183: curl_sin = 15'd12842;
        184: curl_sin = 15'd12891;  185: curl_sin = 15'd12941;  186: curl_sin = 15'd12990;  187: curl_sin = 15'd13039;  188: curl_sin = 15'd13088;  189: curl_sin = 15'd13136;  190: curl_sin = 15'd13184;  191: curl_sin = 15'd13231;
        192: curl_sin = 15'd13279;  193: curl_sin = 15'd13325;  194: curl_sin = 15'd13372;  195: curl_sin = 15'd13418;  196: curl_sin = 15'd13464;  197: curl_sin = 15'd13510;  198: curl_sin = 15'd13555;  199: curl_sin = 15'd13600;
        200: curl_sin = 15'd13645;  201: curl_sin = 15'd13689;  202: curl_sin = 15'd13733;  203: curl_sin = 15'd13777;  204: curl_sin = 15'd13821;  205: curl_sin = 15'd13864;  206: curl_sin = 15'd13906;  207: curl_sin = 15'd13949;
        208: curl_sin = 15'd13991;  209: curl_sin = 15'd14032;  210: curl_sin = 15'd14074;  211: curl_sin = 15'd14115;  212: curl_sin = 15'd14155;  213: curl_sin = 15'd14196;  214: curl_sin = 15'd14236;  215: curl_sin = 15'd14275;
        216: curl_sin = 15'd14315;  217: curl_sin = 15'd14354;  218: curl_sin = 15'd14392;  219: curl_sin = 15'd14430;  220: curl_sin = 15'd14468;  221: curl_sin = 15'd14506;  222: curl_sin = 15'd14543;  223: curl_sin = 15'd14580;
        224: curl_sin = 15'd14616;  225: curl_sin = 15'd14653;  226: curl_sin = 15'd14688;  227: curl_sin = 15'd14724;  228: curl_sin = 15'd14759;  229: curl_sin = 15'd14794;  230: curl_sin = 15'd14828;  231: curl_sin = 15'd14862;
        232: curl_sin = 15'd14896;  233: curl_sin = 15'd14929;  234: curl_sin = 15'd14962;  235: curl_sin = 15'd14995;  236: curl_sin = 15'd15027;  237: curl_sin = 15'd15059;  238: curl_sin = 15'd15090;  239: curl_sin = 15'd15121;
        240: curl_sin = 15'd15152;  241: curl_sin = 15'd15183;  242: curl_sin = 15'd15213;  243: curl_sin = 15'd15242;  244: curl_sin = 15'd15272;  245: curl_sin = 15'd15301;  246: curl_sin = 15'd15329;  247: curl_sin = 15'd15357;
        248: curl_sin = 15'd15385;  249: curl_sin = 15'd15413;  250: curl_sin = 15'd15440;  251: curl_sin = 15'd15466;  252: curl_sin = 15'd15493;  253: curl_sin = 15'd15519;  254: curl_sin = 15'd15544;  255: curl_sin = 15'd15570;
        256: curl_sin = 15'd15594;  257: curl_sin = 15'd15619;  258: curl_sin = 15'd15643;  259: curl_sin = 15'd15667;  260: curl_sin = 15'd15690;  261: curl_sin = 15'd15713;  262: curl_sin = 15'd15736;  263: curl_sin = 15'd15758;
        264: curl_sin = 15'd15780;  265: curl_sin = 15'd15801;  266: curl_sin = 15'd15822;  267: curl_sin = 15'd15843;  268: curl_sin = 15'd15863;  269: curl_sin = 15'd15883;  270: curl_sin = 15'd15903;  271: curl_sin = 15'd15922;
        272: curl_sin = 15'd15941;  273: curl_sin = 15'd15959;  274: curl_sin = 15'd15977;  275: curl_sin = 15'd15995;  276: curl_sin = 15'd16012;  277: curl_sin = 15'd16029;  278: curl_sin = 15'd16045;  279: curl_sin = 15'd16061;
        280: curl_sin = 15'd16077;  281: curl_sin = 15'd16092;  282: curl_sin = 15'd16107;  283: curl_sin = 15'd16122;  284: curl_sin = 15'd16136;  285: curl_sin = 15'd16150;  286: curl_sin = 15'd16163;  287: curl_sin = 15'd16176;
        288: curl_sin = 15'd16189;  289: curl_sin = 15'd16201;  290: curl_sin = 15'd16213;  291: curl_sin = 15'd16224;  292: curl_sin = 15'd16235;  293: curl_sin = 15'd16246;  294: curl_sin = 15'd16256;  295: curl_sin = 15'd16266;
        296: curl_sin = 15'd16275;  297: curl_sin = 15'd16284;  298: curl_sin = 15'd16293;  299: curl_sin = 15'd16301;  300: curl_sin = 15'd16309;  301: curl_sin = 15'd16316;  302: curl_sin = 15'd16324;  303: curl_sin = 15'd16330;
        304: curl_sin = 15'd16337;  305: curl_sin = 15'd16343;  306: curl_sin = 15'd16348;  307: curl_sin = 15'd16353;  308: curl_sin = 15'd16358;  309: curl_sin = 15'd16362;  310: curl_sin = 15'd16366;  311: curl_sin = 15'd16370;
        312: curl_sin = 15'd16373;  313: curl_sin = 15'd16376;  314: curl_sin = 15'd16378;  315: curl_sin = 15'd16380;  316: curl_sin = 15'd16382;  317: curl_sin = 15'd16383;  318: curl_sin = 15'd16384;  319: curl_sin = 15'd16384;
        default: curl_sin = 15'd0;
        endcase
    end
endfunction

// AO 接触阴影强度 = 0.12*g^1.4 * 256，g 归一化 0..1（65 项）
// 长度 65，值域 8 位
function [7:0] curl_ao;
    input [6:0] idx;
    begin
        case (idx)
        0: curl_ao = 8'd0;  1: curl_ao = 8'd0;  2: curl_ao = 8'd0;  3: curl_ao = 8'd0;  4: curl_ao = 8'd1;  5: curl_ao = 8'd1;  6: curl_ao = 8'd1;  7: curl_ao = 8'd1;
        8: curl_ao = 8'd2;  9: curl_ao = 8'd2;  10: curl_ao = 8'd2;  11: curl_ao = 8'd3;  12: curl_ao = 8'd3;  13: curl_ao = 8'd3;  14: curl_ao = 8'd4;  15: curl_ao = 8'd4;
        16: curl_ao = 8'd4;  17: curl_ao = 8'd5;  18: curl_ao = 8'd5;  19: curl_ao = 8'd6;  20: curl_ao = 8'd6;  21: curl_ao = 8'd6;  22: curl_ao = 8'd7;  23: curl_ao = 8'd7;
        24: curl_ao = 8'd8;  25: curl_ao = 8'd8;  26: curl_ao = 8'd9;  27: curl_ao = 8'd9;  28: curl_ao = 8'd10;  29: curl_ao = 8'd10;  30: curl_ao = 8'd11;  31: curl_ao = 8'd11;
        32: curl_ao = 8'd12;  33: curl_ao = 8'd12;  34: curl_ao = 8'd13;  35: curl_ao = 8'd13;  36: curl_ao = 8'd14;  37: curl_ao = 8'd14;  38: curl_ao = 8'd15;  39: curl_ao = 8'd15;
        40: curl_ao = 8'd16;  41: curl_ao = 8'd16;  42: curl_ao = 8'd17;  43: curl_ao = 8'd18;  44: curl_ao = 8'd18;  45: curl_ao = 8'd19;  46: curl_ao = 8'd19;  47: curl_ao = 8'd20;
        48: curl_ao = 8'd21;  49: curl_ao = 8'd21;  50: curl_ao = 8'd22;  51: curl_ao = 8'd22;  52: curl_ao = 8'd23;  53: curl_ao = 8'd24;  54: curl_ao = 8'd24;  55: curl_ao = 8'd25;
        56: curl_ao = 8'd25;  57: curl_ao = 8'd26;  58: curl_ao = 8'd27;  59: curl_ao = 8'd27;  60: curl_ao = 8'd28;  61: curl_ao = 8'd29;  62: curl_ao = 8'd29;  63: curl_ao = 8'd30;
        64: curl_ao = 8'd31;
        default: curl_ao = 8'd0;
        endcase
    end
endfunction

// 下沿投影幂次剖面 x^1.6 * 256（65 项）
// 长度 65，值域 9 位
function [8:0] curl_dr;
    input [6:0] idx;
    begin
        case (idx)
        0: curl_dr = 9'd0;  1: curl_dr = 9'd4;  2: curl_dr = 9'd8;  3: curl_dr = 9'd12;  4: curl_dr = 9'd16;  5: curl_dr = 9'd20;  6: curl_dr = 9'd24;  7: curl_dr = 9'd28;
        8: curl_dr = 9'd32;  9: curl_dr = 9'd36;  10: curl_dr = 9'd40;  11: curl_dr = 9'd44;  12: curl_dr = 9'd48;  13: curl_dr = 9'd52;  14: curl_dr = 9'd56;  15: curl_dr = 9'd60;
        16: curl_dr = 9'd64;  17: curl_dr = 9'd68;  18: curl_dr = 9'd72;  19: curl_dr = 9'd76;  20: curl_dr = 9'd80;  21: curl_dr = 9'd84;  22: curl_dr = 9'd88;  23: curl_dr = 9'd92;
        24: curl_dr = 9'd96;  25: curl_dr = 9'd100;  26: curl_dr = 9'd104;  27: curl_dr = 9'd108;  28: curl_dr = 9'd112;  29: curl_dr = 9'd116;  30: curl_dr = 9'd120;  31: curl_dr = 9'd124;
        32: curl_dr = 9'd128;  33: curl_dr = 9'd132;  34: curl_dr = 9'd136;  35: curl_dr = 9'd140;  36: curl_dr = 9'd144;  37: curl_dr = 9'd148;  38: curl_dr = 9'd152;  39: curl_dr = 9'd156;
        40: curl_dr = 9'd160;  41: curl_dr = 9'd164;  42: curl_dr = 9'd168;  43: curl_dr = 9'd172;  44: curl_dr = 9'd176;  45: curl_dr = 9'd180;  46: curl_dr = 9'd184;  47: curl_dr = 9'd188;
        48: curl_dr = 9'd192;  49: curl_dr = 9'd196;  50: curl_dr = 9'd200;  51: curl_dr = 9'd204;  52: curl_dr = 9'd208;  53: curl_dr = 9'd212;  54: curl_dr = 9'd216;  55: curl_dr = 9'd220;
        56: curl_dr = 9'd224;  57: curl_dr = 9'd228;  58: curl_dr = 9'd232;  59: curl_dr = 9'd236;  60: curl_dr = 9'd240;  61: curl_dr = 9'd244;  62: curl_dr = 9'd248;  63: curl_dr = 9'd252;
        64: curl_dr = 9'd256;
        default: curl_dr = 9'd0;
        endcase
    end
endfunction

// (0.72+0.28*lam)*(1-0.34|u|^3) * 256，u=(i-128)/128（257 项）
// 长度 257，值域 9 位
function [8:0] curl_roll;
    input [8:0] idx;
    begin
        case (idx)
        0: curl_roll = 9'd143;  1: curl_roll = 9'd150;  2: curl_roll = 9'd153;  3: curl_roll = 9'd157;  4: curl_roll = 9'd160;  5: curl_roll = 9'd162;  6: curl_roll = 9'd165;  7: curl_roll = 9'd168;
        8: curl_roll = 9'd170;  9: curl_roll = 9'd173;  10: curl_roll = 9'd175;  11: curl_roll = 9'd177;  12: curl_roll = 9'd179;  13: curl_roll = 9'd182;  14: curl_roll = 9'd184;  15: curl_roll = 9'd186;
        16: curl_roll = 9'd188;  17: curl_roll = 9'd190;  18: curl_roll = 9'd192;  19: curl_roll = 9'd194;  20: curl_roll = 9'd195;  21: curl_roll = 9'd197;  22: curl_roll = 9'd199;  23: curl_roll = 9'd201;
        24: curl_roll = 9'd202;  25: curl_roll = 9'd204;  26: curl_roll = 9'd206;  27: curl_roll = 9'd207;  28: curl_roll = 9'd209;  29: curl_roll = 9'd210;  30: curl_roll = 9'd212;  31: curl_roll = 9'd213;
        32: curl_roll = 9'd215;  33: curl_roll = 9'd216;  34: curl_roll = 9'd217;  35: curl_roll = 9'd219;  36: curl_roll = 9'd220;  37: curl_roll = 9'd221;  38: curl_roll = 9'd223;  39: curl_roll = 9'd224;
        40: curl_roll = 9'd225;  41: curl_roll = 9'd226;  42: curl_roll = 9'd227;  43: curl_roll = 9'd228;  44: curl_roll = 9'd229;  45: curl_roll = 9'd230;  46: curl_roll = 9'd231;  47: curl_roll = 9'd232;
        48: curl_roll = 9'd233;  49: curl_roll = 9'd234;  50: curl_roll = 9'd235;  51: curl_roll = 9'd236;  52: curl_roll = 9'd237;  53: curl_roll = 9'd238;  54: curl_roll = 9'd238;  55: curl_roll = 9'd239;
        56: curl_roll = 9'd240;  57: curl_roll = 9'd241;  58: curl_roll = 9'd241;  59: curl_roll = 9'd242;  60: curl_roll = 9'd243;  61: curl_roll = 9'd243;  62: curl_roll = 9'd244;  63: curl_roll = 9'd244;
        64: curl_roll = 9'd245;  65: curl_roll = 9'd246;  66: curl_roll = 9'd246;  67: curl_roll = 9'd247;  68: curl_roll = 9'd247;  69: curl_roll = 9'd247;  70: curl_roll = 9'd248;  71: curl_roll = 9'd248;
        72: curl_roll = 9'd249;  73: curl_roll = 9'd249;  74: curl_roll = 9'd249;  75: curl_roll = 9'd250;  76: curl_roll = 9'd250;  77: curl_roll = 9'd250;  78: curl_roll = 9'd251;  79: curl_roll = 9'd251;
        80: curl_roll = 9'd251;  81: curl_roll = 9'd251;  82: curl_roll = 9'd252;  83: curl_roll = 9'd252;  84: curl_roll = 9'd252;  85: curl_roll = 9'd252;  86: curl_roll = 9'd252;  87: curl_roll = 9'd253;
        88: curl_roll = 9'd253;  89: curl_roll = 9'd253;  90: curl_roll = 9'd253;  91: curl_roll = 9'd253;  92: curl_roll = 9'd253;  93: curl_roll = 9'd253;  94: curl_roll = 9'd253;  95: curl_roll = 9'd253;
        96: curl_roll = 9'd253;  97: curl_roll = 9'd253;  98: curl_roll = 9'd253;  99: curl_roll = 9'd253;  100: curl_roll = 9'd253;  101: curl_roll = 9'd253;  102: curl_roll = 9'd253;  103: curl_roll = 9'd253;
        104: curl_roll = 9'd253;  105: curl_roll = 9'd253;  106: curl_roll = 9'd253;  107: curl_roll = 9'd253;  108: curl_roll = 9'd252;  109: curl_roll = 9'd252;  110: curl_roll = 9'd252;  111: curl_roll = 9'd252;
        112: curl_roll = 9'd252;  113: curl_roll = 9'd252;  114: curl_roll = 9'd252;  115: curl_roll = 9'd251;  116: curl_roll = 9'd251;  117: curl_roll = 9'd251;  118: curl_roll = 9'd251;  119: curl_roll = 9'd251;
        120: curl_roll = 9'd250;  121: curl_roll = 9'd250;  122: curl_roll = 9'd250;  123: curl_roll = 9'd250;  124: curl_roll = 9'd250;  125: curl_roll = 9'd249;  126: curl_roll = 9'd249;  127: curl_roll = 9'd249;
        128: curl_roll = 9'd249;  129: curl_roll = 9'd248;  130: curl_roll = 9'd248;  131: curl_roll = 9'd248;  132: curl_roll = 9'd248;  133: curl_roll = 9'd247;  134: curl_roll = 9'd247;  135: curl_roll = 9'd247;
        136: curl_roll = 9'd246;  137: curl_roll = 9'd246;  138: curl_roll = 9'd246;  139: curl_roll = 9'd246;  140: curl_roll = 9'd245;  141: curl_roll = 9'd245;  142: curl_roll = 9'd245;  143: curl_roll = 9'd244;
        144: curl_roll = 9'd244;  145: curl_roll = 9'd244;  146: curl_roll = 9'd243;  147: curl_roll = 9'd243;  148: curl_roll = 9'd242;  149: curl_roll = 9'd242;  150: curl_roll = 9'd242;  151: curl_roll = 9'd241;
        152: curl_roll = 9'd241;  153: curl_roll = 9'd240;  154: curl_roll = 9'd240;  155: curl_roll = 9'd240;  156: curl_roll = 9'd239;  157: curl_roll = 9'd239;  158: curl_roll = 9'd238;  159: curl_roll = 9'd238;
        160: curl_roll = 9'd237;  161: curl_roll = 9'd237;  162: curl_roll = 9'd236;  163: curl_roll = 9'd236;  164: curl_roll = 9'd235;  165: curl_roll = 9'd235;  166: curl_roll = 9'd234;  167: curl_roll = 9'd234;
        168: curl_roll = 9'd233;  169: curl_roll = 9'd232;  170: curl_roll = 9'd232;  171: curl_roll = 9'd231;  172: curl_roll = 9'd230;  173: curl_roll = 9'd230;  174: curl_roll = 9'd229;  175: curl_roll = 9'd228;
        176: curl_roll = 9'd228;  177: curl_roll = 9'd227;  178: curl_roll = 9'd226;  179: curl_roll = 9'd226;  180: curl_roll = 9'd225;  181: curl_roll = 9'd224;  182: curl_roll = 9'd223;  183: curl_roll = 9'd222;
        184: curl_roll = 9'd222;  185: curl_roll = 9'd221;  186: curl_roll = 9'd220;  187: curl_roll = 9'd219;  188: curl_roll = 9'd218;  189: curl_roll = 9'd217;  190: curl_roll = 9'd216;  191: curl_roll = 9'd215;
        192: curl_roll = 9'd215;  193: curl_roll = 9'd214;  194: curl_roll = 9'd213;  195: curl_roll = 9'd212;  196: curl_roll = 9'd211;  197: curl_roll = 9'd209;  198: curl_roll = 9'd208;  199: curl_roll = 9'd207;
        200: curl_roll = 9'd206;  201: curl_roll = 9'd205;  202: curl_roll = 9'd204;  203: curl_roll = 9'd203;  204: curl_roll = 9'd202;  205: curl_roll = 9'd200;  206: curl_roll = 9'd199;  207: curl_roll = 9'd198;
        208: curl_roll = 9'd197;  209: curl_roll = 9'd195;  210: curl_roll = 9'd194;  211: curl_roll = 9'd193;  212: curl_roll = 9'd192;  213: curl_roll = 9'd190;  214: curl_roll = 9'd189;  215: curl_roll = 9'd187;
        216: curl_roll = 9'd186;  217: curl_roll = 9'd185;  218: curl_roll = 9'd183;  219: curl_roll = 9'd182;  220: curl_roll = 9'd180;  221: curl_roll = 9'd179;  222: curl_roll = 9'd177;  223: curl_roll = 9'd175;
        224: curl_roll = 9'd174;  225: curl_roll = 9'd172;  226: curl_roll = 9'd171;  227: curl_roll = 9'd169;  228: curl_roll = 9'd167;  229: curl_roll = 9'd165;  230: curl_roll = 9'd164;  231: curl_roll = 9'd162;
        232: curl_roll = 9'd160;  233: curl_roll = 9'd158;  234: curl_roll = 9'd157;  235: curl_roll = 9'd155;  236: curl_roll = 9'd153;  237: curl_roll = 9'd151;  238: curl_roll = 9'd149;  239: curl_roll = 9'd147;
        240: curl_roll = 9'd145;  241: curl_roll = 9'd143;  242: curl_roll = 9'd141;  243: curl_roll = 9'd139;  244: curl_roll = 9'd138;  245: curl_roll = 9'd136;  246: curl_roll = 9'd135;  247: curl_roll = 9'd134;
        248: curl_roll = 9'd133;  249: curl_roll = 9'd131;  250: curl_roll = 9'd130;  251: curl_roll = 9'd129;  252: curl_roll = 9'd127;  253: curl_roll = 9'd126;  254: curl_roll = 9'd125;  255: curl_roll = 9'd123;
        256: curl_roll = 9'd122;
        default: curl_roll = 9'd0;
        endcase
    end
endfunction

// 折边高光 0.16*exp(-((e-0.35)/1.80)^2) * 256，e=-2.5+i/16（104 项）
// 长度 104，值域 7 位
function [6:0] curl_gauss;
    input [6:0] idx;
    begin
        case (idx)
        0: curl_gauss = 7'd3;  1: curl_gauss = 7'd4;  2: curl_gauss = 7'd4;  3: curl_gauss = 7'd5;  4: curl_gauss = 7'd5;  5: curl_gauss = 7'd6;  6: curl_gauss = 7'd6;  7: curl_gauss = 7'd7;
        8: curl_gauss = 7'd7;  9: curl_gauss = 7'd8;  10: curl_gauss = 7'd9;  11: curl_gauss = 7'd10;  12: curl_gauss = 7'd11;  13: curl_gauss = 7'd11;  14: curl_gauss = 7'd12;  15: curl_gauss = 7'd13;
        16: curl_gauss = 7'd14;  17: curl_gauss = 7'd15;  18: curl_gauss = 7'd16;  19: curl_gauss = 7'd17;  20: curl_gauss = 7'd19;  21: curl_gauss = 7'd20;  22: curl_gauss = 7'd21;  23: curl_gauss = 7'd22;
        24: curl_gauss = 7'd23;  25: curl_gauss = 7'd25;  26: curl_gauss = 7'd26;  27: curl_gauss = 7'd27;  28: curl_gauss = 7'd28;  29: curl_gauss = 7'd29;  30: curl_gauss = 7'd31;  31: curl_gauss = 7'd32;
        32: curl_gauss = 7'd33;  33: curl_gauss = 7'd34;  34: curl_gauss = 7'd35;  35: curl_gauss = 7'd36;  36: curl_gauss = 7'd37;  37: curl_gauss = 7'd37;  38: curl_gauss = 7'd38;  39: curl_gauss = 7'd39;
        40: curl_gauss = 7'd39;  41: curl_gauss = 7'd40;  42: curl_gauss = 7'd40;  43: curl_gauss = 7'd41;  44: curl_gauss = 7'd41;  45: curl_gauss = 7'd41;  46: curl_gauss = 7'd41;  47: curl_gauss = 7'd41;
        48: curl_gauss = 7'd41;  49: curl_gauss = 7'd40;  50: curl_gauss = 7'd40;  51: curl_gauss = 7'd40;  52: curl_gauss = 7'd39;  53: curl_gauss = 7'd38;  54: curl_gauss = 7'd38;  55: curl_gauss = 7'd37;
        56: curl_gauss = 7'd36;  57: curl_gauss = 7'd35;  58: curl_gauss = 7'd34;  59: curl_gauss = 7'd33;  60: curl_gauss = 7'd32;  61: curl_gauss = 7'd31;  62: curl_gauss = 7'd30;  63: curl_gauss = 7'd28;
        64: curl_gauss = 7'd27;  65: curl_gauss = 7'd26;  66: curl_gauss = 7'd25;  67: curl_gauss = 7'd24;  68: curl_gauss = 7'd22;  69: curl_gauss = 7'd21;  70: curl_gauss = 7'd20;  71: curl_gauss = 7'd19;
        72: curl_gauss = 7'd18;  73: curl_gauss = 7'd17;  74: curl_gauss = 7'd15;  75: curl_gauss = 7'd14;  76: curl_gauss = 7'd13;  77: curl_gauss = 7'd12;  78: curl_gauss = 7'd12;  79: curl_gauss = 7'd11;
        80: curl_gauss = 7'd10;  81: curl_gauss = 7'd9;  82: curl_gauss = 7'd8;  83: curl_gauss = 7'd8;  84: curl_gauss = 7'd7;  85: curl_gauss = 7'd6;  86: curl_gauss = 7'd6;  87: curl_gauss = 7'd5;
        88: curl_gauss = 7'd5;  89: curl_gauss = 7'd4;  90: curl_gauss = 7'd4;  91: curl_gauss = 7'd3;  92: curl_gauss = 7'd3;  93: curl_gauss = 7'd3;  94: curl_gauss = 7'd2;  95: curl_gauss = 7'd2;
        96: curl_gauss = 7'd2;  97: curl_gauss = 7'd2;  98: curl_gauss = 7'd1;  99: curl_gauss = 7'd1;  100: curl_gauss = 7'd1;  101: curl_gauss = 7'd1;  102: curl_gauss = 7'd1;  103: curl_gauss = 7'd1;
        default: curl_gauss = 7'd0;
        endcase
    end
endfunction

// ---- 帧表：逐帧几何（每帧查一次，不是逐像素）----
// 内容接缝行，恒偶数 ⇒ read 通路 split_len = split*640 必为 256 整数倍
// 长度 70，值域 9 位
function [8:0] curl_split;
    input [6:0] idx;
    begin
        case (idx)
        0: curl_split = 9'd480;  1: curl_split = 9'd462;  2: curl_split = 9'd446;  3: curl_split = 9'd432;  4: curl_split = 9'd416;  5: curl_split = 9'd402;  6: curl_split = 9'd388;  7: curl_split = 9'd374;
        8: curl_split = 9'd360;  9: curl_split = 9'd346;  10: curl_split = 9'd334;  11: curl_split = 9'd322;  12: curl_split = 9'd308;  13: curl_split = 9'd296;  14: curl_split = 9'd286;  15: curl_split = 9'd274;
        16: curl_split = 9'd264;  17: curl_split = 9'd252;  18: curl_split = 9'd242;  19: curl_split = 9'd232;  20: curl_split = 9'd222;  21: curl_split = 9'd214;  22: curl_split = 9'd204;  23: curl_split = 9'd196;
        24: curl_split = 9'd186;  25: curl_split = 9'd178;  26: curl_split = 9'd170;  27: curl_split = 9'd162;  28: curl_split = 9'd156;  29: curl_split = 9'd148;  30: curl_split = 9'd140;  31: curl_split = 9'd134;
        32: curl_split = 9'd128;  33: curl_split = 9'd120;  34: curl_split = 9'd114;  35: curl_split = 9'd108;  36: curl_split = 9'd102;  37: curl_split = 9'd98;  38: curl_split = 9'd92;  39: curl_split = 9'd86;
        40: curl_split = 9'd82;  41: curl_split = 9'd76;  42: curl_split = 9'd72;  43: curl_split = 9'd68;  44: curl_split = 9'd62;  45: curl_split = 9'd58;  46: curl_split = 9'd54;  47: curl_split = 9'd50;
        48: curl_split = 9'd46;  49: curl_split = 9'd42;  50: curl_split = 9'd40;  51: curl_split = 9'd36;  52: curl_split = 9'd32;  53: curl_split = 9'd30;  54: curl_split = 9'd26;  55: curl_split = 9'd22;
        56: curl_split = 9'd20;  57: curl_split = 9'd16;  58: curl_split = 9'd14;  59: curl_split = 9'd12;  60: curl_split = 9'd8;  61: curl_split = 9'd6;  62: curl_split = 9'd2;  63: curl_split = 9'd0;
        64: curl_split = 9'd0;  65: curl_split = 9'd0;  66: curl_split = 9'd0;  67: curl_split = 9'd0;  68: curl_split = 9'd0;  69: curl_split = 9'd0;
        default: curl_split = 9'd0;
        endcase
    end
endfunction

// s*256，20 位【补码】（负值已加 2^20）
// 长度 70，值域 20 位
function [19:0] curl_s8;
    input [6:0] idx;
    begin
        case (idx)
        0: curl_s8 = 20'd122880;  1: curl_s8 = 20'd118716;  2: curl_s8 = 20'd114654;  3: curl_s8 = 20'd110693;  4: curl_s8 = 20'd106831;  5: curl_s8 = 20'd103066;  6: curl_s8 = 20'd99398;  7: curl_s8 = 20'd95824;
        8: curl_s8 = 20'd92343;  9: curl_s8 = 20'd88954;  10: curl_s8 = 20'd85655;  11: curl_s8 = 20'd82444;  12: curl_s8 = 20'd79321;  13: curl_s8 = 20'd76283;  14: curl_s8 = 20'd73329;  15: curl_s8 = 20'd70458;
        16: curl_s8 = 20'd67668;  17: curl_s8 = 20'd64957;  18: curl_s8 = 20'd62324;  19: curl_s8 = 20'd59768;  20: curl_s8 = 20'd57288;  21: curl_s8 = 20'd54880;  22: curl_s8 = 20'd52545;  23: curl_s8 = 20'd50280;
        24: curl_s8 = 20'd48085;  25: curl_s8 = 20'd45957;  26: curl_s8 = 20'd43895;  27: curl_s8 = 20'd41897;  28: curl_s8 = 20'd39963;  29: curl_s8 = 20'd38090;  30: curl_s8 = 20'd36277;  31: curl_s8 = 20'd34523;
        32: curl_s8 = 20'd32826;  33: curl_s8 = 20'd31184;  34: curl_s8 = 20'd29596;  35: curl_s8 = 20'd28061;  36: curl_s8 = 20'd26577;  37: curl_s8 = 20'd25143;  38: curl_s8 = 20'd23756;  39: curl_s8 = 20'd22417;
        40: curl_s8 = 20'd21122;  41: curl_s8 = 20'd19871;  42: curl_s8 = 20'd18661;  43: curl_s8 = 20'd17493;  44: curl_s8 = 20'd16363;  45: curl_s8 = 20'd15271;  46: curl_s8 = 20'd14215;  47: curl_s8 = 20'd13193;
        48: curl_s8 = 20'd12205;  49: curl_s8 = 20'd11247;  50: curl_s8 = 20'd10320;  51: curl_s8 = 20'd9422;  52: curl_s8 = 20'd8551;  53: curl_s8 = 20'd7705;  54: curl_s8 = 20'd6883;  55: curl_s8 = 20'd6083;
        56: curl_s8 = 20'd5305;  57: curl_s8 = 20'd4546;  58: curl_s8 = 20'd3805;  59: curl_s8 = 20'd3081;  60: curl_s8 = 20'd2372;  61: curl_s8 = 20'd1676;  62: curl_s8 = 20'd992;  63: curl_s8 = 20'd319;
        64: curl_s8 = 20'd1048231;  65: curl_s8 = 20'd1047574;  66: curl_s8 = 20'd1046923;  67: curl_s8 = 20'd1046277;  68: curl_s8 = 20'd1045634;  69: curl_s8 = 20'd1044992;
        default: curl_s8 = 20'd0;
        endcase
    end
endfunction

// r*256，卷脊半径
// 长度 70，值域 13 位
function [12:0] curl_rad8;
    input [6:0] idx;
    begin
        case (idx)
        0: curl_rad8 = 13'd0;  1: curl_rad8 = 13'd2529;  2: curl_rad8 = 13'd3555;  3: curl_rad8 = 13'd4327;  4: curl_rad8 = 13'd4965;  5: curl_rad8 = 13'd5517;  6: curl_rad8 = 13'd5632;  7: curl_rad8 = 13'd5632;
        8: curl_rad8 = 13'd5632;  9: curl_rad8 = 13'd5632;  10: curl_rad8 = 13'd5632;  11: curl_rad8 = 13'd5632;  12: curl_rad8 = 13'd5632;  13: curl_rad8 = 13'd5632;  14: curl_rad8 = 13'd5632;  15: curl_rad8 = 13'd5632;
        16: curl_rad8 = 13'd5632;  17: curl_rad8 = 13'd5632;  18: curl_rad8 = 13'd5632;  19: curl_rad8 = 13'd5632;  20: curl_rad8 = 13'd5632;  21: curl_rad8 = 13'd5632;  22: curl_rad8 = 13'd5632;  23: curl_rad8 = 13'd5632;
        24: curl_rad8 = 13'd5632;  25: curl_rad8 = 13'd5632;  26: curl_rad8 = 13'd5632;  27: curl_rad8 = 13'd5632;  28: curl_rad8 = 13'd5632;  29: curl_rad8 = 13'd5632;  30: curl_rad8 = 13'd5632;  31: curl_rad8 = 13'd5632;
        32: curl_rad8 = 13'd5632;  33: curl_rad8 = 13'd5632;  34: curl_rad8 = 13'd5632;  35: curl_rad8 = 13'd5632;  36: curl_rad8 = 13'd5632;  37: curl_rad8 = 13'd5632;  38: curl_rad8 = 13'd5632;  39: curl_rad8 = 13'd5632;
        40: curl_rad8 = 13'd5632;  41: curl_rad8 = 13'd5632;  42: curl_rad8 = 13'd5632;  43: curl_rad8 = 13'd5632;  44: curl_rad8 = 13'd5632;  45: curl_rad8 = 13'd5632;  46: curl_rad8 = 13'd5632;  47: curl_rad8 = 13'd5632;
        48: curl_rad8 = 13'd5632;  49: curl_rad8 = 13'd5632;  50: curl_rad8 = 13'd5632;  51: curl_rad8 = 13'd5632;  52: curl_rad8 = 13'd5632;  53: curl_rad8 = 13'd5632;  54: curl_rad8 = 13'd5632;  55: curl_rad8 = 13'd5632;
        56: curl_rad8 = 13'd5632;  57: curl_rad8 = 13'd5632;  58: curl_rad8 = 13'd5632;  59: curl_rad8 = 13'd5632;  60: curl_rad8 = 13'd5632;  61: curl_rad8 = 13'd5632;  62: curl_rad8 = 13'd5632;  63: curl_rad8 = 13'd5632;
        64: curl_rad8 = 13'd5632;  65: curl_rad8 = 13'd5632;  66: curl_rad8 = 13'd5632;  67: curl_rad8 = 13'd5632;  68: curl_rad8 = 13'd5632;  69: curl_rad8 = 13'd5632;
        default: curl_rad8 = 13'd0;
        endcase
    end
endfunction

// lift*256 = 弓形抬高量（逐列再乘 sin）
// 长度 70，值域 14 位
function [13:0] curl_lift8;
    input [6:0] idx;
    begin
        case (idx)
        0: curl_lift8 = 14'd0;  1: curl_lift8 = 14'd4932;  2: curl_lift8 = 14'd6931;  3: curl_lift8 = 14'd8437;  4: curl_lift8 = 14'd9682;  5: curl_lift8 = 14'd10758;  6: curl_lift8 = 14'd10982;  7: curl_lift8 = 14'd10982;
        8: curl_lift8 = 14'd10982;  9: curl_lift8 = 14'd10982;  10: curl_lift8 = 14'd10982;  11: curl_lift8 = 14'd10982;  12: curl_lift8 = 14'd10982;  13: curl_lift8 = 14'd10982;  14: curl_lift8 = 14'd10982;  15: curl_lift8 = 14'd10982;
        16: curl_lift8 = 14'd10982;  17: curl_lift8 = 14'd10982;  18: curl_lift8 = 14'd10982;  19: curl_lift8 = 14'd10982;  20: curl_lift8 = 14'd10982;  21: curl_lift8 = 14'd10982;  22: curl_lift8 = 14'd10982;  23: curl_lift8 = 14'd10982;
        24: curl_lift8 = 14'd10982;  25: curl_lift8 = 14'd10982;  26: curl_lift8 = 14'd10982;  27: curl_lift8 = 14'd10982;  28: curl_lift8 = 14'd10982;  29: curl_lift8 = 14'd10982;  30: curl_lift8 = 14'd10982;  31: curl_lift8 = 14'd10982;
        32: curl_lift8 = 14'd10982;  33: curl_lift8 = 14'd10982;  34: curl_lift8 = 14'd10982;  35: curl_lift8 = 14'd10982;  36: curl_lift8 = 14'd10982;  37: curl_lift8 = 14'd10982;  38: curl_lift8 = 14'd10982;  39: curl_lift8 = 14'd10982;
        40: curl_lift8 = 14'd10982;  41: curl_lift8 = 14'd10982;  42: curl_lift8 = 14'd10982;  43: curl_lift8 = 14'd10982;  44: curl_lift8 = 14'd10982;  45: curl_lift8 = 14'd10982;  46: curl_lift8 = 14'd10982;  47: curl_lift8 = 14'd10982;
        48: curl_lift8 = 14'd10982;  49: curl_lift8 = 14'd10982;  50: curl_lift8 = 14'd10982;  51: curl_lift8 = 14'd10982;  52: curl_lift8 = 14'd10982;  53: curl_lift8 = 14'd10982;  54: curl_lift8 = 14'd10982;  55: curl_lift8 = 14'd10982;
        56: curl_lift8 = 14'd10982;  57: curl_lift8 = 14'd10982;  58: curl_lift8 = 14'd10982;  59: curl_lift8 = 14'd10982;  60: curl_lift8 = 14'd10982;  61: curl_lift8 = 14'd10982;  62: curl_lift8 = 14'd10982;  63: curl_lift8 = 14'd10982;
        64: curl_lift8 = 14'd10982;  65: curl_lift8 = 14'd10982;  66: curl_lift8 = 14'd10982;  67: curl_lift8 = 14'd10982;  68: curl_lift8 = 14'd10982;  69: curl_lift8 = 14'd10982;
        default: curl_lift8 = 14'd0;
        endcase
    end
endfunction

// 128*65536/rad8，逐像素算 u 索引用，避免除法
// 长度 70，值域 18 位
function [17:0] curl_inv_r;
    input [6:0] idx;
    begin
        case (idx)
        0: curl_inv_r = 18'd0;  1: curl_inv_r = 18'd3317;  2: curl_inv_r = 18'd2360;  3: curl_inv_r = 18'd1939;  4: curl_inv_r = 18'd1690;  5: curl_inv_r = 18'd1521;  6: curl_inv_r = 18'd1489;  7: curl_inv_r = 18'd1489;
        8: curl_inv_r = 18'd1489;  9: curl_inv_r = 18'd1489;  10: curl_inv_r = 18'd1489;  11: curl_inv_r = 18'd1489;  12: curl_inv_r = 18'd1489;  13: curl_inv_r = 18'd1489;  14: curl_inv_r = 18'd1489;  15: curl_inv_r = 18'd1489;
        16: curl_inv_r = 18'd1489;  17: curl_inv_r = 18'd1489;  18: curl_inv_r = 18'd1489;  19: curl_inv_r = 18'd1489;  20: curl_inv_r = 18'd1489;  21: curl_inv_r = 18'd1489;  22: curl_inv_r = 18'd1489;  23: curl_inv_r = 18'd1489;
        24: curl_inv_r = 18'd1489;  25: curl_inv_r = 18'd1489;  26: curl_inv_r = 18'd1489;  27: curl_inv_r = 18'd1489;  28: curl_inv_r = 18'd1489;  29: curl_inv_r = 18'd1489;  30: curl_inv_r = 18'd1489;  31: curl_inv_r = 18'd1489;
        32: curl_inv_r = 18'd1489;  33: curl_inv_r = 18'd1489;  34: curl_inv_r = 18'd1489;  35: curl_inv_r = 18'd1489;  36: curl_inv_r = 18'd1489;  37: curl_inv_r = 18'd1489;  38: curl_inv_r = 18'd1489;  39: curl_inv_r = 18'd1489;
        40: curl_inv_r = 18'd1489;  41: curl_inv_r = 18'd1489;  42: curl_inv_r = 18'd1489;  43: curl_inv_r = 18'd1489;  44: curl_inv_r = 18'd1489;  45: curl_inv_r = 18'd1489;  46: curl_inv_r = 18'd1489;  47: curl_inv_r = 18'd1489;
        48: curl_inv_r = 18'd1489;  49: curl_inv_r = 18'd1489;  50: curl_inv_r = 18'd1489;  51: curl_inv_r = 18'd1489;  52: curl_inv_r = 18'd1489;  53: curl_inv_r = 18'd1489;  54: curl_inv_r = 18'd1489;  55: curl_inv_r = 18'd1489;
        56: curl_inv_r = 18'd1489;  57: curl_inv_r = 18'd1489;  58: curl_inv_r = 18'd1489;  59: curl_inv_r = 18'd1489;  60: curl_inv_r = 18'd1489;  61: curl_inv_r = 18'd1489;  62: curl_inv_r = 18'd1489;  63: curl_inv_r = 18'd1489;
        64: curl_inv_r = 18'd1489;  65: curl_inv_r = 18'd1489;  66: curl_inv_r = 18'd1489;  67: curl_inv_r = 18'd1489;  68: curl_inv_r = 18'd1489;  69: curl_inv_r = 18'd1489;
        default: curl_inv_r = 18'd0;
        endcase
    end
endfunction

// 总帧数
localparam CURL_N = 70;
