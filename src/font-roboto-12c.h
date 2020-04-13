#include <Arduino.h>

const static uint8_t Roboto_Condensed_12[] PROGMEM = {
	0x09, // Width: 9
	0x11, // Height: 17
	0x20, // First Char: 32
	0xE0, // Numbers of Chars: 224

	// Jump Table:
	0xFF, 0xFF, 0x00, 0x08,  // 32:65535
	0x00, 0x00, 0x0E, 0x08,  // 33:0
	0x00, 0x0E, 0x0D, 0x08,  // 34:14
	0x00, 0x1B, 0x17, 0x08,  // 35:27
	0x00, 0x32, 0x14, 0x08,  // 36:50
	0x00, 0x46, 0x17, 0x08,  // 37:70
	0x00, 0x5D, 0x17, 0x08,  // 38:93
	0x00, 0x74, 0x0A, 0x08,  // 39:116
	0x00, 0x7E, 0x11, 0x08,  // 40:126
	0x00, 0x8F, 0x0E, 0x08,  // 41:143
	0x00, 0x9D, 0x13, 0x08,  // 42:157
	0x00, 0xB0, 0x17, 0x08,  // 43:176
	0x00, 0xC7, 0x0B, 0x08,  // 44:199
	0x00, 0xD2, 0x14, 0x08,  // 45:210
	0x00, 0xE6, 0x0E, 0x08,  // 46:230
	0x00, 0xF4, 0x13, 0x08,  // 47:244
	0x01, 0x07, 0x14, 0x08,  // 48:263
	0x01, 0x1B, 0x0E, 0x08,  // 49:283
	0x01, 0x29, 0x14, 0x08,  // 50:297
	0x01, 0x3D, 0x14, 0x08,  // 51:317
	0x01, 0x51, 0x17, 0x08,  // 52:337
	0x01, 0x68, 0x14, 0x08,  // 53:360
	0x01, 0x7C, 0x14, 0x08,  // 54:380
	0x01, 0x90, 0x13, 0x08,  // 55:400
	0x01, 0xA3, 0x17, 0x08,  // 56:419
	0x01, 0xBA, 0x14, 0x08,  // 57:442
	0x01, 0xCE, 0x0E, 0x08,  // 58:462
	0x01, 0xDC, 0x0E, 0x08,  // 59:476
	0x01, 0xEA, 0x14, 0x08,  // 60:490
	0x01, 0xFE, 0x14, 0x08,  // 61:510
	0x02, 0x12, 0x14, 0x08,  // 62:530
	0x02, 0x26, 0x13, 0x08,  // 63:550
	0x02, 0x39, 0x17, 0x08,  // 64:569
	0x02, 0x50, 0x17, 0x08,  // 65:592
	0x02, 0x67, 0x14, 0x08,  // 66:615
	0x02, 0x7B, 0x14, 0x08,  // 67:635
	0x02, 0x8F, 0x17, 0x08,  // 68:655
	0x02, 0xA6, 0x14, 0x08,  // 69:678
	0x02, 0xBA, 0x13, 0x08,  // 70:698
	0x02, 0xCD, 0x14, 0x08,  // 71:717
	0x02, 0xE1, 0x14, 0x08,  // 72:737
	0x02, 0xF5, 0x14, 0x08,  // 73:757
	0x03, 0x09, 0x14, 0x08,  // 74:777
	0x03, 0x1D, 0x17, 0x08,  // 75:797
	0x03, 0x34, 0x14, 0x08,  // 76:820
	0x03, 0x48, 0x14, 0x08,  // 77:840
	0x03, 0x5C, 0x14, 0x08,  // 78:860
	0x03, 0x70, 0x14, 0x08,  // 79:880
	0x03, 0x84, 0x16, 0x08,  // 80:900
	0x03, 0x9A, 0x17, 0x08,  // 81:922
	0x03, 0xB1, 0x17, 0x08,  // 82:945
	0x03, 0xC8, 0x17, 0x08,  // 83:968
	0x03, 0xDF, 0x16, 0x08,  // 84:991
	0x03, 0xF5, 0x14, 0x08,  // 85:1013
	0x04, 0x09, 0x16, 0x08,  // 86:1033
	0x04, 0x1F, 0x16, 0x08,  // 87:1055
	0x04, 0x35, 0x17, 0x08,  // 88:1077
	0x04, 0x4C, 0x16, 0x08,  // 89:1100
	0x04, 0x62, 0x14, 0x08,  // 90:1122
	0x04, 0x76, 0x0E, 0x08,  // 91:1142
	0x04, 0x84, 0x11, 0x08,  // 92:1156
	0x04, 0x95, 0x0E, 0x08,  // 93:1173
	0x04, 0xA3, 0x13, 0x08,  // 94:1187
	0x04, 0xB6, 0x14, 0x08,  // 95:1206
	0x04, 0xCA, 0x0D, 0x08,  // 96:1226
	0x04, 0xD7, 0x14, 0x08,  // 97:1239
	0x04, 0xEB, 0x14, 0x08,  // 98:1259
	0x04, 0xFF, 0x14, 0x08,  // 99:1279
	0x05, 0x13, 0x14, 0x08,  // 100:1299
	0x05, 0x27, 0x14, 0x08,  // 101:1319
	0x05, 0x3B, 0x16, 0x08,  // 102:1339
	0x05, 0x51, 0x14, 0x08,  // 103:1361
	0x05, 0x65, 0x14, 0x08,  // 104:1381
	0x05, 0x79, 0x14, 0x08,  // 105:1401
	0x05, 0x8D, 0x11, 0x08,  // 106:1421
	0x05, 0x9E, 0x14, 0x08,  // 107:1438
	0x05, 0xB2, 0x14, 0x08,  // 108:1458
	0x05, 0xC6, 0x17, 0x08,  // 109:1478
	0x05, 0xDD, 0x14, 0x08,  // 110:1501
	0x05, 0xF1, 0x14, 0x08,  // 111:1521
	0x06, 0x05, 0x14, 0x08,  // 112:1541
	0x06, 0x19, 0x14, 0x08,  // 113:1561
	0x06, 0x2D, 0x13, 0x08,  // 114:1581
	0x06, 0x40, 0x14, 0x08,  // 115:1600
	0x06, 0x54, 0x14, 0x08,  // 116:1620
	0x06, 0x68, 0x14, 0x08,  // 117:1640
	0x06, 0x7C, 0x16, 0x08,  // 118:1660
	0x06, 0x92, 0x17, 0x08,  // 119:1682
	0x06, 0xA9, 0x17, 0x08,  // 120:1705
	0x06, 0xC0, 0x16, 0x08,  // 121:1728
	0x06, 0xD6, 0x17, 0x08,  // 122:1750
	0x06, 0xED, 0x14, 0x08,  // 123:1773
	0x07, 0x01, 0x0E, 0x08,  // 124:1793
	0x07, 0x0F, 0x14, 0x08,  // 125:1807
	0x07, 0x23, 0x17, 0x08,  // 126:1827
	0xFF, 0xFF, 0x00, 0x08,  // 127:65535
	0xFF, 0xFF, 0x00, 0x08,  // 128:65535
	0xFF, 0xFF, 0x00, 0x08,  // 129:65535
	0xFF, 0xFF, 0x00, 0x08,  // 130:65535
	0xFF, 0xFF, 0x00, 0x08,  // 131:65535
	0xFF, 0xFF, 0x00, 0x08,  // 132:65535
	0xFF, 0xFF, 0x00, 0x08,  // 133:65535
	0xFF, 0xFF, 0x00, 0x08,  // 134:65535
	0xFF, 0xFF, 0x00, 0x08,  // 135:65535
	0xFF, 0xFF, 0x00, 0x08,  // 136:65535
	0xFF, 0xFF, 0x00, 0x08,  // 137:65535
	0xFF, 0xFF, 0x00, 0x08,  // 138:65535
	0xFF, 0xFF, 0x00, 0x08,  // 139:65535
	0xFF, 0xFF, 0x00, 0x08,  // 140:65535
	0xFF, 0xFF, 0x00, 0x08,  // 141:65535
	0xFF, 0xFF, 0x00, 0x08,  // 142:65535
	0xFF, 0xFF, 0x00, 0x08,  // 143:65535
	0xFF, 0xFF, 0x00, 0x08,  // 144:65535
	0xFF, 0xFF, 0x00, 0x08,  // 145:65535
	0xFF, 0xFF, 0x00, 0x08,  // 146:65535
	0xFF, 0xFF, 0x00, 0x08,  // 147:65535
	0xFF, 0xFF, 0x00, 0x08,  // 148:65535
	0xFF, 0xFF, 0x00, 0x08,  // 149:65535
	0xFF, 0xFF, 0x00, 0x08,  // 150:65535
	0xFF, 0xFF, 0x00, 0x08,  // 151:65535
	0xFF, 0xFF, 0x00, 0x08,  // 152:65535
	0xFF, 0xFF, 0x00, 0x08,  // 153:65535
	0xFF, 0xFF, 0x00, 0x08,  // 154:65535
	0xFF, 0xFF, 0x00, 0x08,  // 155:65535
	0xFF, 0xFF, 0x00, 0x08,  // 156:65535
	0xFF, 0xFF, 0x00, 0x08,  // 157:65535
	0xFF, 0xFF, 0x00, 0x08,  // 158:65535
	0xFF, 0xFF, 0x00, 0x08,  // 159:65535
	0xFF, 0xFF, 0x00, 0x08,  // 160:65535
	0x07, 0x3A, 0x0E, 0x08,  // 161:1850
	0x07, 0x48, 0x14, 0x08,  // 162:1864
	0x07, 0x5C, 0x17, 0x08,  // 163:1884
	0x07, 0x73, 0x17, 0x08,  // 164:1907
	0x07, 0x8A, 0x16, 0x08,  // 165:1930
	0x07, 0xA0, 0x0E, 0x08,  // 166:1952
	0x07, 0xAE, 0x17, 0x08,  // 167:1966
	0x07, 0xC5, 0x10, 0x08,  // 168:1989
	0x07, 0xD5, 0x17, 0x08,  // 169:2005
	0x07, 0xEC, 0x10, 0x08,  // 170:2028
	0x07, 0xFC, 0x11, 0x08,  // 171:2044
	0x08, 0x0D, 0x14, 0x08,  // 172:2061
	0x08, 0x21, 0x14, 0x08,  // 173:2081
	0x08, 0x35, 0x17, 0x08,  // 174:2101
	0x08, 0x4C, 0x10, 0x08,  // 175:2124
	0x08, 0x5C, 0x10, 0x08,  // 176:2140
	0x08, 0x6C, 0x14, 0x08,  // 177:2156
	0x08, 0x80, 0x10, 0x08,  // 178:2176
	0x08, 0x90, 0x10, 0x08,  // 179:2192
	0x08, 0xA0, 0x0D, 0x08,  // 180:2208
	0x08, 0xAD, 0x14, 0x08,  // 181:2221
	0x08, 0xC1, 0x11, 0x08,  // 182:2241
	0x08, 0xD2, 0x0D, 0x08,  // 183:2258
	0x08, 0xDF, 0x0E, 0x08,  // 184:2271
	0x08, 0xED, 0x0D, 0x08,  // 185:2285
	0x08, 0xFA, 0x10, 0x08,  // 186:2298
	0x09, 0x0A, 0x11, 0x08,  // 187:2314
	0x09, 0x1B, 0x17, 0x08,  // 188:2331
	0x09, 0x32, 0x17, 0x08,  // 189:2354
	0x09, 0x49, 0x17, 0x08,  // 190:2377
	0x09, 0x60, 0x14, 0x08,  // 191:2400
	0x09, 0x74, 0x17, 0x08,  // 192:2420
	0x09, 0x8B, 0x17, 0x08,  // 193:2443
	0x09, 0xA2, 0x17, 0x08,  // 194:2466
	0x09, 0xB9, 0x17, 0x08,  // 195:2489
	0x09, 0xD0, 0x17, 0x08,  // 196:2512
	0x09, 0xE7, 0x17, 0x08,  // 197:2535
	0x09, 0xFE, 0x17, 0x08,  // 198:2558
	0x0A, 0x15, 0x14, 0x08,  // 199:2581
	0x0A, 0x29, 0x14, 0x08,  // 200:2601
	0x0A, 0x3D, 0x14, 0x08,  // 201:2621
	0x0A, 0x51, 0x14, 0x08,  // 202:2641
	0x0A, 0x65, 0x14, 0x08,  // 203:2661
	0x0A, 0x79, 0x14, 0x08,  // 204:2681
	0x0A, 0x8D, 0x14, 0x08,  // 205:2701
	0x0A, 0xA1, 0x14, 0x08,  // 206:2721
	0x0A, 0xB5, 0x14, 0x08,  // 207:2741
	0x0A, 0xC9, 0x17, 0x09,  // 208:2761
	0x0A, 0xE0, 0x14, 0x08,  // 209:2784
	0x0A, 0xF4, 0x14, 0x08,  // 210:2804
	0x0B, 0x08, 0x14, 0x08,  // 211:2824
	0x0B, 0x1C, 0x14, 0x08,  // 212:2844
	0x0B, 0x30, 0x14, 0x08,  // 213:2864
	0x0B, 0x44, 0x14, 0x08,  // 214:2884
	0x0B, 0x58, 0x14, 0x08,  // 215:2904
	0x0B, 0x6C, 0x17, 0x08,  // 216:2924
	0x0B, 0x83, 0x14, 0x08,  // 217:2947
	0x0B, 0x97, 0x14, 0x08,  // 218:2967
	0x0B, 0xAB, 0x14, 0x08,  // 219:2987
	0x0B, 0xBF, 0x14, 0x08,  // 220:3007
	0x0B, 0xD3, 0x16, 0x08,  // 221:3027
	0x0B, 0xE9, 0x14, 0x08,  // 222:3049
	0x0B, 0xFD, 0x14, 0x08,  // 223:3069
	0x0C, 0x11, 0x14, 0x08,  // 224:3089
	0x0C, 0x25, 0x14, 0x08,  // 225:3109
	0x0C, 0x39, 0x14, 0x08,  // 226:3129
	0x0C, 0x4D, 0x14, 0x08,  // 227:3149
	0x0C, 0x61, 0x14, 0x08,  // 228:3169
	0x0C, 0x75, 0x14, 0x08,  // 229:3189
	0x0C, 0x89, 0x17, 0x08,  // 230:3209
	0x0C, 0xA0, 0x14, 0x08,  // 231:3232
	0x0C, 0xB4, 0x14, 0x08,  // 232:3252
	0x0C, 0xC8, 0x14, 0x08,  // 233:3272
	0x0C, 0xDC, 0x14, 0x08,  // 234:3292
	0x0C, 0xF0, 0x14, 0x08,  // 235:3312
	0x0D, 0x04, 0x14, 0x08,  // 236:3332
	0x0D, 0x18, 0x14, 0x08,  // 237:3352
	0x0D, 0x2C, 0x14, 0x08,  // 238:3372
	0x0D, 0x40, 0x14, 0x08,  // 239:3392
	0x0D, 0x54, 0x14, 0x08,  // 240:3412
	0x0D, 0x68, 0x14, 0x08,  // 241:3432
	0x0D, 0x7C, 0x14, 0x08,  // 242:3452
	0x0D, 0x90, 0x14, 0x08,  // 243:3472
	0x0D, 0xA4, 0x14, 0x08,  // 244:3492
	0x0D, 0xB8, 0x14, 0x08,  // 245:3512
	0x0D, 0xCC, 0x14, 0x08,  // 246:3532
	0x0D, 0xE0, 0x17, 0x08,  // 247:3552
	0x0D, 0xF7, 0x14, 0x08,  // 248:3575
	0x0E, 0x0B, 0x14, 0x08,  // 249:3595
	0x0E, 0x1F, 0x14, 0x08,  // 250:3615
	0x0E, 0x33, 0x14, 0x08,  // 251:3635
	0x0E, 0x47, 0x14, 0x08,  // 252:3655
	0x0E, 0x5B, 0x16, 0x08,  // 253:3675
	0x0E, 0x71, 0x14, 0x08,  // 254:3697
	0x0E, 0x85, 0x16, 0x08,  // 255:3717

	// Font Data:
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x13,	// 33
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x00,0x00,0x1C,	// 34
	0x00,0x02,0x00,0x20,0x02,0x00,0xE0,0x1F,0x00,0x3C,0x02,0x00,0x20,0x12,0x00,0xE0,0x0F,0x00,0x3C,0x02,0x00,0x20,0x02,	// 35
	0x00,0x00,0x00,0x30,0x0C,0x00,0x48,0x10,0x00,0x84,0x10,0x00,0x82,0x20,0x00,0x84,0x10,0x00,0x18,0x0F,	// 36
	0x38,0x00,0x00,0x44,0x00,0x00,0x44,0x06,0x00,0x38,0x01,0x00,0xC0,0x04,0x00,0x30,0x1A,0x00,0x08,0x11,0x00,0x00,0x1E,	// 37
	0x00,0x00,0x00,0x00,0x0F,0x00,0xF8,0x10,0x00,0xC4,0x10,0x00,0x24,0x11,0x00,0x18,0x16,0x00,0x00,0x08,0x00,0x00,0x17,	// 38
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,	// 39
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x3F,0x00,0x08,0xC0,0x00,0x04,0x80,	// 40
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0xC0,0x00,0xF0,0x3F,	// 41
	0x00,0x00,0x00,0x10,0x00,0x00,0x90,0x00,0x00,0x50,0x00,0x00,0x3C,0x00,0x00,0x50,0x00,0x00,0x90,	// 42
	0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0xE0,0x0F,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,	// 43
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,	// 44
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,	// 45
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,	// 46
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x0E,0x00,0xC0,0x01,0x00,0x38,0x00,0x00,0x04,	// 47
	0x00,0x00,0x00,0xF0,0x07,0x00,0x08,0x0A,0x00,0x04,0x11,0x00,0x84,0x10,0x00,0x44,0x10,0x00,0xF8,0x0F,	// 48
	0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x08,0x00,0x00,0xFC,0x1F,	// 49
	0x00,0x00,0x00,0x18,0x10,0x00,0x04,0x1C,0x00,0x04,0x12,0x00,0x04,0x11,0x00,0xC8,0x10,0x00,0x30,0x10,	// 50
	0x00,0x00,0x00,0x18,0x0C,0x00,0x04,0x10,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0xC4,0x10,0x00,0x38,0x0F,	// 51
	0x00,0x00,0x00,0x00,0x03,0x00,0xC0,0x02,0x00,0x20,0x02,0x00,0x18,0x02,0x00,0xFC,0x1F,0x00,0x00,0x02,0x00,0x00,0x02,	// 52
	0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x0C,0x00,0x44,0x10,0x00,0x44,0x10,0x00,0x44,0x10,0x00,0x84,0x0F,	// 53
	0x00,0x00,0x00,0xE0,0x0F,0x00,0x98,0x10,0x00,0x44,0x10,0x00,0x44,0x10,0x00,0x44,0x10,0x00,0x80,0x0F,	// 54
	0x00,0x00,0x00,0x04,0x00,0x00,0x04,0x10,0x00,0x04,0x0C,0x00,0x84,0x03,0x00,0x64,0x00,0x00,0x1C,	// 55
	0x00,0x00,0x00,0x30,0x0E,0x00,0x48,0x11,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x48,0x11,0x00,0x30,0x0E,	// 56
	0x00,0x00,0x00,0xF8,0x00,0x00,0x04,0x11,0x00,0x04,0x12,0x00,0x04,0x12,0x00,0x04,0x0A,0x00,0xF8,0x07,	// 57
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x10,	// 58
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x70,	// 59
	0x00,0x00,0x00,0x00,0x01,0x00,0x80,0x02,0x00,0x80,0x02,0x00,0x40,0x02,0x00,0x40,0x04,0x00,0x40,0x04,	// 60
	0x00,0x00,0x00,0x40,0x02,0x00,0x40,0x02,0x00,0x40,0x02,0x00,0x40,0x02,0x00,0x40,0x02,0x00,0x40,0x02,	// 61
	0x00,0x00,0x00,0x40,0x04,0x00,0x40,0x04,0x00,0x40,0x04,0x00,0x80,0x02,0x00,0x80,0x02,0x00,0x00,0x01,	// 62
	0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x00,0x04,0x00,0x00,0x04,0x13,0x00,0x84,0x00,0x00,0x78,	// 63
	0xE0,0x07,0x00,0x10,0x08,0x00,0xC8,0x13,0x00,0x24,0x14,0x00,0x14,0x12,0x00,0xE4,0x13,0x00,0x04,0x04,0x00,0xF8,0x03,	// 64
	0x00,0x00,0x00,0x00,0x1C,0x00,0xC0,0x03,0x00,0x38,0x02,0x00,0x0C,0x02,0x00,0xF0,0x02,0x00,0x00,0x07,0x00,0x00,0x18,	// 65
	0x00,0x00,0x00,0xFC,0x1F,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x78,0x0F,	// 66
	0x00,0x00,0x00,0xF0,0x0F,0x00,0x08,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x18,0x0C,	// 67
	0x00,0x00,0x00,0xFC,0x1F,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x18,0x0C,0x00,0xE0,0x03,	// 68
	0x00,0x00,0x00,0x00,0x00,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x84,0x10,	// 69
	0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x1F,0x00,0x84,0x00,0x00,0x84,0x00,0x00,0x84,0x00,0x00,0x84,	// 70
	0x00,0x00,0x00,0xF8,0x07,0x00,0x04,0x08,0x00,0x04,0x10,0x00,0x04,0x11,0x00,0x04,0x11,0x00,0x18,0x1F,	// 71
	0x00,0x00,0x00,0xFC,0x1F,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0xFC,0x1F,	// 72
	0x00,0x00,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0xFC,0x1F,0x00,0x04,0x10,0x00,0x04,0x10,	// 73
	0x00,0x00,0x00,0x00,0x0C,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0xFC,0x0F,	// 74
	0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x1F,0x00,0xC0,0x00,0x00,0x20,0x01,0x00,0x10,0x06,0x00,0x08,0x08,0x00,0x04,0x10,	// 75
	0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x1F,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,	// 76
	0x00,0x00,0x00,0xFC,0x1F,0x00,0x30,0x00,0x00,0xC0,0x00,0x00,0x80,0x01,0x00,0x60,0x00,0x00,0xFC,0x1F,	// 77
	0x00,0x00,0x00,0xFC,0x1F,0x00,0x10,0x00,0x00,0x60,0x00,0x00,0x80,0x01,0x00,0x00,0x06,0x00,0xFC,0x1F,	// 78
	0x00,0x00,0x00,0xF8,0x0F,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0xF8,0x0F,	// 79
	0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x1F,0x00,0x04,0x01,0x00,0x04,0x01,0x00,0x04,0x01,0x00,0x84,0x00,0x00,0x78,	// 80
	0x00,0x00,0x00,0xF8,0x0F,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x08,0x28,0x00,0xF0,0x47,	// 81
	0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x1F,0x00,0x04,0x01,0x00,0x04,0x01,0x00,0x04,0x03,0x00,0xD8,0x0C,0x00,0x20,0x10,	// 82
	0x00,0x00,0x00,0x38,0x0C,0x00,0x44,0x10,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x08,0x11,0x00,0x10,0x0E,	// 83
	0x00,0x00,0x00,0x04,0x00,0x00,0x04,0x00,0x00,0x04,0x00,0x00,0xFC,0x1F,0x00,0x04,0x00,0x00,0x04,0x00,0x00,0x04,	// 84
	0x00,0x00,0x00,0xFC,0x0F,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0xFC,0x0F,	// 85
	0x00,0x00,0x00,0x3C,0x00,0x00,0xC0,0x01,0x00,0x00,0x06,0x00,0x00,0x1C,0x00,0x80,0x03,0x00,0x78,0x00,0x00,0x04,	// 86
	0x00,0x00,0x00,0xFC,0x03,0x00,0x00,0x1C,0x00,0xF0,0x03,0x00,0x3C,0x00,0x00,0xC0,0x03,0x00,0x00,0x1F,0x00,0xFC,	// 87
	0x00,0x00,0x00,0x04,0x18,0x00,0x18,0x04,0x00,0x20,0x03,0x00,0xC0,0x01,0x00,0x30,0x02,0x00,0x08,0x0C,0x00,0x04,0x10,	// 88
	0x00,0x00,0x00,0x0C,0x00,0x00,0x30,0x00,0x00,0xC0,0x00,0x00,0x80,0x1F,0x00,0x60,0x00,0x00,0x18,0x00,0x00,0x04,	// 89
	0x00,0x00,0x00,0x04,0x18,0x00,0x04,0x16,0x00,0x84,0x11,0x00,0x44,0x10,0x00,0x34,0x10,0x00,0x0C,0x10,	// 90
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xFF,0x00,0x02,0x80,	// 91
	0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x00,0x00,0x70,0x00,0x00,0x80,0x03,0x00,0x00,0x1C,	// 92
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x80,0x00,0xFE,0xFF,	// 93
	0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0x18,0x00,0x00,0x0C,0x00,0x00,0x70,0x00,0x00,0x80,	// 94
	0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0x20,	// 95
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x08,	// 96
	0x00,0x00,0x00,0x40,0x0E,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0xC0,0x1F,	// 97
	0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x1F,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xC0,0x0F,	// 98
	0x00,0x00,0x00,0x80,0x07,0x00,0x40,0x08,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xC0,0x08,	// 99
	0x00,0x00,0x00,0xC0,0x0F,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xFC,0x1F,	// 100
	0x00,0x00,0x00,0x80,0x07,0x00,0x40,0x09,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0xC0,0x11,	// 101
	0x00,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0xF8,0x1F,0x00,0x24,0x00,0x00,0x22,0x00,0x00,0x22,0x00,0x00,0x02,	// 102
	0x00,0x00,0x00,0xC0,0x4F,0x00,0x20,0x90,0x00,0x20,0x90,0x00,0x20,0x90,0x00,0x20,0x90,0x00,0xE0,0x7F,	// 103
	0x00,0x00,0x00,0xFC,0x1F,0x00,0x40,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0xC0,0x1F,	// 104
	0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xE4,0x1F,0x00,0x00,0x10,0x00,0x00,0x10,	// 105
	0x00,0x00,0x00,0x00,0x80,0x00,0x20,0x80,0x00,0x20,0x80,0x00,0x20,0x40,0x00,0xE4,0x3F,	// 106
	0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x1F,0x00,0x00,0x01,0x00,0x80,0x02,0x00,0x40,0x0C,0x00,0x20,0x10,	// 107
	0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0xFC,0x1F,0x00,0x00,0x10,0x00,0x00,0x10,	// 108
	0x00,0x00,0x00,0xE0,0x1F,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0xC0,0x1F,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0xE0,0x1F,	// 109
	0x00,0x00,0x00,0xE0,0x1F,0x00,0x40,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0xC0,0x1F,	// 110
	0x00,0x00,0x00,0xC0,0x0F,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xC0,0x0F,	// 111
	0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xC0,0x0F,	// 112
	0x00,0x00,0x00,0xC0,0x0F,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xE0,0xFF,	// 113
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x1F,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0x20,	// 114
	0x00,0x00,0x00,0xC0,0x08,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0x20,0x12,0x00,0x40,0x0E,	// 115
	0x00,0x00,0x00,0x20,0x00,0x00,0x20,0x00,0x00,0xF8,0x0F,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,	// 116
	0x00,0x00,0x00,0xE0,0x0F,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0xE0,0x1F,	// 117
	0x00,0x00,0x00,0x60,0x00,0x00,0x80,0x03,0x00,0x00,0x0C,0x00,0x00,0x18,0x00,0x00,0x07,0x00,0xC0,0x00,0x00,0x20,	// 118
	0x00,0x00,0x00,0xE0,0x07,0x00,0x00,0x1C,0x00,0x80,0x03,0x00,0xE0,0x00,0x00,0x00,0x07,0x00,0x00,0x1E,0x00,0xE0,0x01,	// 119
	0x00,0x00,0x00,0x20,0x10,0x00,0x40,0x08,0x00,0x80,0x04,0x00,0x00,0x03,0x00,0x80,0x04,0x00,0x60,0x18,0x00,0x00,0x10,	// 120
	0x00,0x00,0x00,0xE0,0x80,0x00,0x00,0x83,0x00,0x00,0x6C,0x00,0x00,0x18,0x00,0x00,0x06,0x00,0xC0,0x01,0x00,0x20,	// 121
	0x00,0x00,0x00,0x20,0x10,0x00,0x20,0x1C,0x00,0x20,0x12,0x00,0x20,0x11,0x00,0xE0,0x10,0x00,0x20,0x10,0x00,0x00,0x10,	// 122
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0xFC,0x3E,0x00,0x02,0x40,0x00,0x02,0x40,	// 123
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x3F,	// 124
	0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x80,0x00,0x02,0x80,0x00,0xFC,0x7E,0x00,0x00,0x01,0x00,0x00,0x01,	// 125
	0x00,0x03,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0x00,0x01,0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x80,0x01,	// 126
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0xFF,	// 161
	0x00,0x00,0x00,0x80,0x07,0x00,0x40,0x08,0x00,0x20,0x10,0x00,0x18,0x20,0x00,0x20,0x10,0x00,0xC0,0x08,	// 162
	0x00,0x00,0x00,0x80,0x10,0x00,0xF8,0x1F,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x18,0x10,	// 163
	0x00,0x00,0x00,0xD0,0x1F,0x00,0x20,0x10,0x00,0x10,0x10,0x00,0x10,0x10,0x00,0x10,0x10,0x00,0x20,0x08,0x00,0xD0,0x17,	// 164
	0x00,0x00,0x00,0x0C,0x05,0x00,0x30,0x05,0x00,0xC0,0x05,0x00,0x80,0x1F,0x00,0x60,0x05,0x00,0x18,0x05,0x00,0x04,	// 165
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x3E,	// 166
	0x00,0x00,0x00,0xB8,0xE7,0x00,0x44,0x08,0x01,0x44,0x08,0x01,0x84,0x08,0x01,0x84,0x10,0x01,0x08,0xB9,0x00,0x10,0x46,	// 167
	0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,	// 168
	0x00,0x00,0x00,0xC0,0x0F,0x00,0xA0,0x17,0x00,0x60,0x18,0x00,0x60,0x18,0x00,0xA0,0x14,0x00,0x40,0x08,0x00,0x80,0x07,	// 169
	0x00,0x00,0x00,0x00,0x00,0x00,0x48,0x00,0x00,0xA4,0x00,0x00,0xA4,0x00,0x00,0xFC,	// 170
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x80,0x0C,0x00,0x40,0x03,0x00,0xC0,0x0C,	// 171
	0x00,0x00,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0x80,0x03,	// 172
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,	// 173
	0x00,0x00,0x00,0xC0,0x0F,0x00,0x20,0x10,0x00,0xE0,0x17,0x00,0x60,0x12,0x00,0xA0,0x15,0x00,0x40,0x08,0x00,0x80,0x07,	// 174
	0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x04,0x00,0x00,0x04,0x00,0x00,0x04,	// 175
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x00,0x00,0x14,0x00,0x00,0x18,	// 176
	0x00,0x00,0x00,0x80,0x10,0x00,0x80,0x10,0x00,0x80,0x10,0x00,0xF0,0x17,0x00,0x80,0x10,0x00,0x80,0x10,	// 177
	0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0xC4,0x00,0x00,0xA4,0x00,0x00,0x9C,	// 178
	0x00,0x00,0x00,0x00,0x00,0x00,0x48,0x00,0x00,0x84,0x00,0x00,0xA4,0x00,0x00,0x7C,	// 179
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x04,	// 180
	0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0xE0,0x1F,	// 181
	0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x00,0x00,0xFC,0x01,0x00,0xFC,0x01,0x00,0xFC,0x1F,	// 182
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,	// 183
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA0,0x00,0x00,0xC0,	// 184
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0xFC,	// 185
	0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0x00,0x84,0x00,0x00,0x84,0x00,0x00,0x7C,	// 186
	0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x05,0x00,0xC0,0x0A,0x00,0x00,0x07,	// 187
	0x08,0x00,0x00,0x08,0x00,0x00,0xF0,0x08,0x00,0x00,0x07,0x00,0xC0,0x0C,0x00,0x30,0x0A,0x00,0x00,0x1F,0x00,0x00,0x08,	// 188
	0x08,0x00,0x00,0x08,0x00,0x00,0xF0,0x08,0x00,0x00,0x06,0x00,0xC0,0x11,0x00,0x30,0x19,0x00,0x00,0x15,0x00,0x00,0x12,	// 189
	0x88,0x00,0x00,0xA8,0x00,0x00,0xA8,0x08,0x00,0x58,0x07,0x00,0xC0,0x0C,0x00,0x30,0x0A,0x00,0x00,0x1F,0x00,0x00,0x08,	// 190
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0x00,0x84,0x00,0x20,0x83,0x00,0x00,0x80,0x00,0x00,0x60,	// 191
	0x00,0x00,0x00,0x00,0x1C,0x00,0xC0,0x03,0x00,0x39,0x02,0x00,0x0E,0x02,0x00,0xF0,0x02,0x00,0x00,0x07,0x00,0x00,0x18,	// 192
	0x00,0x00,0x00,0x00,0x1C,0x00,0xC0,0x03,0x00,0x38,0x02,0x00,0x0E,0x02,0x00,0xF1,0x02,0x00,0x00,0x07,0x00,0x00,0x18,	// 193
	0x00,0x00,0x00,0x00,0x1C,0x00,0xC0,0x03,0x00,0x3A,0x02,0x00,0x0D,0x02,0x00,0xF2,0x02,0x00,0x00,0x07,0x00,0x00,0x18,	// 194
	0x00,0x00,0x00,0x00,0x1C,0x00,0xC3,0x03,0x00,0x39,0x02,0x00,0x0E,0x02,0x00,0xF2,0x02,0x00,0x01,0x07,0x00,0x00,0x18,	// 195
	0x00,0x00,0x00,0x00,0x1C,0x00,0xC1,0x03,0x00,0x38,0x02,0x00,0x0C,0x02,0x00,0xF1,0x02,0x00,0x00,0x07,0x00,0x00,0x18,	// 196
	0x00,0x00,0x00,0x00,0x1C,0x00,0xC0,0x03,0x00,0x3A,0x02,0x00,0x0D,0x02,0x00,0xF2,0x02,0x00,0x00,0x07,0x00,0x00,0x18,	// 197
	0x00,0x00,0x00,0x00,0x1C,0x00,0xC0,0x03,0x00,0x30,0x02,0x00,0xFC,0x1F,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x04,0x10,	// 198
	0x00,0x00,0x00,0xF0,0x0F,0x00,0x08,0x10,0x00,0x04,0xB0,0x00,0x04,0xA0,0x00,0x04,0x50,0x00,0x18,0x0C,	// 199
	0x00,0x00,0x00,0x00,0x00,0x00,0x85,0x10,0x00,0x82,0x10,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x84,0x10,	// 200
	0x00,0x00,0x00,0x00,0x00,0x00,0x84,0x10,0x00,0x82,0x10,0x00,0x85,0x10,0x00,0x84,0x10,0x00,0x84,0x10,	// 201
	0x00,0x00,0x00,0x00,0x00,0x00,0x82,0x10,0x00,0x85,0x10,0x00,0x82,0x10,0x00,0x84,0x10,0x00,0x84,0x10,	// 202
	0x00,0x00,0x00,0xFD,0x1F,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x85,0x10,0x00,0x84,0x10,0x00,0x84,0x10,	// 203
	0x00,0x00,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x05,0x10,0x00,0xFE,0x1F,0x00,0x04,0x10,0x00,0x04,0x10,	// 204
	0x00,0x00,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0xFE,0x1F,0x00,0x05,0x10,0x00,0x04,0x10,	// 205
	0x00,0x00,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x02,0x10,0x00,0xFD,0x1F,0x00,0x02,0x10,0x00,0x04,0x10,	// 206
	0x00,0x00,0x00,0x04,0x10,0x00,0x05,0x10,0x00,0x04,0x10,0x00,0xFC,0x1F,0x00,0x05,0x10,0x00,0x04,0x10,	// 207
	0x80,0x00,0x00,0xFC,0x1F,0x00,0x84,0x10,0x00,0x84,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x08,0x08,0x00,0xF0,0x07,	// 208
	0x00,0x00,0x00,0xFC,0x1F,0x00,0x13,0x00,0x00,0x61,0x00,0x00,0x82,0x01,0x00,0x02,0x06,0x00,0xFD,0x1F,	// 209
	0x00,0x00,0x00,0xF8,0x0F,0x00,0x04,0x10,0x00,0x05,0x10,0x00,0x02,0x10,0x00,0x04,0x10,0x00,0xF8,0x0F,	// 210
	0x00,0x00,0x00,0xF8,0x0F,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x02,0x10,0x00,0x05,0x10,0x00,0xF8,0x0F,	// 211
	0x00,0x00,0x00,0xF8,0x0F,0x00,0x04,0x10,0x00,0x06,0x10,0x00,0x05,0x10,0x00,0x06,0x10,0x00,0xF8,0x0F,	// 212
	0x00,0x00,0x00,0xF8,0x0F,0x00,0x07,0x10,0x00,0x05,0x10,0x00,0x06,0x10,0x00,0x06,0x10,0x00,0xF9,0x0F,	// 213
	0x00,0x00,0x00,0xF8,0x0F,0x00,0x05,0x10,0x00,0x04,0x10,0x00,0x04,0x10,0x00,0x05,0x10,0x00,0xF8,0x0F,	// 214
	0x00,0x00,0x00,0x80,0x10,0x00,0x00,0x09,0x00,0x00,0x06,0x00,0x00,0x06,0x00,0x00,0x09,0x00,0x80,0x10,	// 215
	0x00,0x00,0x00,0xF8,0x17,0x00,0x04,0x1C,0x00,0x04,0x13,0x00,0x84,0x10,0x00,0x64,0x10,0x00,0x18,0x0C,0x00,0xE4,0x03,	// 216
	0x00,0x00,0x00,0xFC,0x0F,0x00,0x01,0x10,0x00,0x02,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0xFC,0x0F,	// 217
	0x00,0x00,0x00,0xFC,0x0F,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x02,0x10,0x00,0x01,0x10,0x00,0xFC,0x0F,	// 218
	0x00,0x00,0x00,0xFC,0x0F,0x00,0x00,0x10,0x00,0x02,0x10,0x00,0x01,0x10,0x00,0x02,0x10,0x00,0xFC,0x0F,	// 219
	0x00,0x00,0x00,0xFC,0x0F,0x00,0x01,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x01,0x10,0x00,0xFC,0x0F,	// 220
	0x00,0x00,0x00,0x0C,0x00,0x00,0x30,0x00,0x00,0xC2,0x00,0x00,0x81,0x1F,0x00,0x60,0x00,0x00,0x18,0x00,0x00,0x04,	// 221
	0x00,0x00,0x00,0xFC,0x1F,0x00,0x20,0x04,0x00,0x20,0x04,0x00,0x20,0x04,0x00,0x20,0x04,0x00,0xC0,0x03,	// 222
	0x00,0x00,0x00,0xF8,0x1F,0x00,0x04,0x00,0x00,0x04,0x10,0x00,0x44,0x10,0x00,0xB8,0x11,0x00,0x00,0x1E,	// 223
	0x00,0x00,0x00,0x40,0x0E,0x00,0x20,0x11,0x00,0x24,0x11,0x00,0x28,0x11,0x00,0x20,0x11,0x00,0xC0,0x1F,	// 224
	0x00,0x00,0x00,0x40,0x0E,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0x28,0x11,0x00,0x24,0x11,0x00,0xC0,0x1F,	// 225
	0x00,0x00,0x00,0x40,0x0E,0x00,0x20,0x11,0x00,0x28,0x11,0x00,0x24,0x11,0x00,0x28,0x11,0x00,0xC0,0x1F,	// 226
	0x00,0x00,0x00,0x40,0x0E,0x00,0x2C,0x11,0x00,0x24,0x11,0x00,0x28,0x11,0x00,0x28,0x11,0x00,0xC4,0x1F,	// 227
	0x00,0x00,0x00,0x40,0x0E,0x00,0x24,0x11,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0x24,0x11,0x00,0xC0,0x1F,	// 228
	0x00,0x00,0x00,0x40,0x0E,0x00,0x20,0x11,0x00,0x28,0x11,0x00,0x14,0x11,0x00,0x28,0x11,0x00,0xC0,0x1F,	// 229
	0x40,0x0E,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0x20,0x19,0x00,0xC0,0x0F,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0xC0,0x11,	// 230
	0x00,0x00,0x00,0x80,0x07,0x00,0x40,0x08,0x00,0x20,0xB0,0x00,0x20,0xA0,0x00,0x20,0x50,0x00,0xC0,0x08,	// 231
	0x00,0x00,0x00,0x80,0x07,0x00,0x40,0x09,0x00,0x24,0x11,0x00,0x28,0x11,0x00,0x20,0x11,0x00,0xC0,0x11,	// 232
	0x00,0x00,0x00,0x80,0x07,0x00,0x40,0x09,0x00,0x20,0x11,0x00,0x28,0x11,0x00,0x24,0x11,0x00,0xC0,0x11,	// 233
	0x00,0x00,0x00,0x80,0x07,0x00,0x40,0x09,0x00,0x28,0x11,0x00,0x24,0x11,0x00,0x28,0x11,0x00,0xC0,0x11,	// 234
	0x00,0x00,0x00,0x80,0x07,0x00,0x44,0x09,0x00,0x20,0x11,0x00,0x20,0x11,0x00,0x24,0x11,0x00,0xC0,0x11,	// 235
	0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x10,0x00,0x24,0x10,0x00,0xE8,0x1F,0x00,0x00,0x10,0x00,0x00,0x10,	// 236
	0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xE8,0x1F,0x00,0x04,0x10,0x00,0x00,0x10,	// 237
	0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x10,0x00,0x28,0x10,0x00,0xE4,0x1F,0x00,0x08,0x10,0x00,0x00,0x10,	// 238
	0x00,0x00,0x00,0x00,0x00,0x00,0x24,0x10,0x00,0x20,0x10,0x00,0xE0,0x1F,0x00,0x04,0x10,0x00,0x00,0x10,	// 239
	0x00,0x00,0x00,0x80,0x0F,0x00,0x44,0x10,0x00,0x54,0x10,0x00,0x48,0x10,0x00,0x58,0x10,0x00,0xE4,0x0F,	// 240
	0x00,0x00,0x00,0xE0,0x1F,0x00,0x4C,0x00,0x00,0x24,0x00,0x00,0x28,0x00,0x00,0x28,0x00,0x00,0xC4,0x1F,	// 241
	0x00,0x00,0x00,0xC0,0x0F,0x00,0x24,0x10,0x00,0x28,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xC0,0x0F,	// 242
	0x00,0x00,0x00,0xC0,0x0F,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x28,0x10,0x00,0x24,0x10,0x00,0xC0,0x0F,	// 243
	0x00,0x00,0x00,0xC0,0x0F,0x00,0x20,0x10,0x00,0x28,0x10,0x00,0x24,0x10,0x00,0x28,0x10,0x00,0xC0,0x0F,	// 244
	0x00,0x00,0x00,0xC0,0x0F,0x00,0x2C,0x10,0x00,0x24,0x10,0x00,0x28,0x10,0x00,0x28,0x10,0x00,0xC4,0x0F,	// 245
	0x00,0x00,0x00,0xC0,0x0F,0x00,0x24,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x24,0x10,0x00,0xC0,0x0F,	// 246
	0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x20,0x09,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,0x01,	// 247
	0x00,0x00,0x00,0xC0,0x2F,0x00,0x20,0x18,0x00,0x20,0x16,0x00,0xA0,0x11,0x00,0x70,0x10,0x00,0xC0,0x0F,	// 248
	0x00,0x00,0x00,0xE0,0x0F,0x00,0x04,0x10,0x00,0x08,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0xE0,0x1F,	// 249
	0x00,0x00,0x00,0xE0,0x0F,0x00,0x00,0x10,0x00,0x08,0x10,0x00,0x04,0x10,0x00,0x00,0x10,0x00,0xE0,0x1F,	// 250
	0x00,0x00,0x00,0xE0,0x0F,0x00,0x00,0x10,0x00,0x08,0x10,0x00,0x04,0x10,0x00,0x08,0x10,0x00,0xE0,0x1F,	// 251
	0x00,0x00,0x00,0xE0,0x0F,0x00,0x04,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x04,0x10,0x00,0xE0,0x1F,	// 252
	0x00,0x00,0x00,0xE0,0x80,0x00,0x00,0x83,0x00,0x00,0x6C,0x00,0x08,0x18,0x00,0x04,0x06,0x00,0xC0,0x01,0x00,0x20,	// 253
	0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0x20,0x10,0x00,0xC0,0x0F,	// 254
	0x00,0x00,0x00,0xE0,0x80,0x00,0x04,0x83,0x00,0x00,0x6C,0x00,0x00,0x18,0x00,0x04,0x06,0x00,0xC0,0x01,0x00,0x20	// 255
};