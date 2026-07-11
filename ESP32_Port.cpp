#include "fabgl.h"
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>

#undef word

extern "C" {
  #ifndef _ARDUINO_H_
  #define _ARDUINO_H_
  #endif
  #include "MSX.h"
  
  // MSX.c에서 호출하는 함수 연결
  void SetColor(byte N, byte R, byte G, byte B);
}
#include "config.h"

// [중요] VGA16Controller 유지
extern fabgl::VGA16Controller VGAController;
extern fabgl::PS2Controller PS2Controller;
extern fabgl::Canvas *Canvas;

// 화면 버퍼
fabgl::Bitmap *msxScreen = nullptr;
uint8_t *XBuf = NULL;

static fabgl::VGA16Controller* vga16 = nullptr;

// Devuelve el puntero a VGA16Controller, enlazándolo bajo demanda si aún no
// se ha hecho. Centraliza el patrón "if(!vga16) vga16 = &VGAController;"
// que antes se repetía en varios sitios.
static inline fabgl::VGA16Controller* ensureVga16() {
  if (!vga16) vga16 = &VGAController;
  return vga16;
}

// === 색상 보정 설정 (정상 작동 확인됨) ===
#define COLOR_MAP_RGB  // 변환 없음 (표준 RGB 배열 - 빨간색/녹색/파란색 정상)

fabgl::RGB888 convertColor(byte R, byte G, byte B) {
  fabgl::RGB888 color;
  
  #if defined(COLOR_MAP_BGR)
    color.R = B; color.G = G; color.B = R;
  #elif defined(COLOR_MAP_RGB)
    color.R = R; color.G = G; color.B = B;
  #elif defined(COLOR_MAP_RBG)
    color.R = R; color.G = B; color.B = G; 
  #elif defined(COLOR_MAP_GRB)
    color.R = G; color.G = R; color.B = B;
  #else 
    color.R = R; color.G = G; color.B = B;
  #endif
  
  return color;
}


// ============================================================================
// Mapeo de teclado -> matriz MSX (activa a nivel bajo / Active Low)
// ============================================================================
//
// CAUSA RAIZ DEL BUG ORIGINAL:
// FabGL resuelve el estado de SHIFT/CAPSLOCK dentro del propio driver de
// teclado (Keyboard::manageCAPSLOCK / Keyboard::VKtoAlternateVK en
// keyboard.cpp) ANTES de entregarnos la virtual key. Esto significa que:
//
//   - Una letra SIN mayúsculas llega como VK_a..VK_z (minúscula)
//   - La misma letra CON SHIFT llega como VK_A..VK_Z (mayúscula)
//     -> son dos virtual keys distintas, no una sola con un flag de shift.
//   - Un símbolo con SHIFT llega como una virtual key DERIVADA distinta:
//     VK_1->VK_EXCLAIM, VK_QUOTE->VK_QUOTEDBL, VK_SLASH->VK_QUESTION, etc.
//
// El switch original solo contemplaba VK_A..VK_Z (mayúsculas) y los símbolos
// "base" sin SHIFT. Por eso:
//   - Las minúsculas (que es como llegan las letras SIN shift) no coincidían
//     con ningún case -> no se reconocía la letra en absoluto sin Shift.
//   - Los símbolos con SHIFT (! @ # $ % ^ & * ( ) _ + : " < > ? { } | ~)
//     tampoco tenían case propio -> se perdían.
//
// LA SOLUCIÓN no es tratar el shift a mano: ambas variantes (base y
// derivada) deben apuntar a la MISMA posición row/bit de la matriz MSX,
// porque es la propia BIOS del MSX la que decide mayúscula/minúscula/símbolo
// en función del bit de SHIFT (fila 6, bit 0x01), que ya se envía aparte.
//
// Se sustituyen los dos switch (PRESS/RELEASE) —duplicados y ahora también
// desincronizables— por una única tabla de datos recorrida por una sola
// función. Así solo hay un sitio donde añadir/corregir una tecla.
// ============================================================================

static inline void PressKey(uint8_t row, uint8_t mask) {
  KeyState[row] &= ~mask;
}

static inline void ReleaseKey(uint8_t row, uint8_t mask) {
  KeyState[row] |= mask;
}

struct KeyMapEntry {
  fabgl::VirtualKey vk;
  uint8_t row;
  uint8_t mask;
};

static constexpr KeyMapEntry KeyMap[] = {
  // --- Fila 0: 0-7 (+ símbolos derivados de SHIFT, + teclado numérico) ---
  { fabgl::VK_0, 0, 0x01 }, { fabgl::VK_RIGHTPAREN, 0, 0x01 }, { fabgl::VK_KP_0, 0, 0x01 },
  { fabgl::VK_1, 0, 0x02 }, { fabgl::VK_EXCLAIM,    0, 0x02 }, { fabgl::VK_KP_1, 0, 0x02 },
  { fabgl::VK_2, 0, 0x04 }, { fabgl::VK_AT,         0, 0x04 }, { fabgl::VK_KP_2, 0, 0x04 },
  { fabgl::VK_3, 0, 0x08 }, { fabgl::VK_HASH,       0, 0x08 }, { fabgl::VK_KP_3, 0, 0x08 },
  { fabgl::VK_4, 0, 0x10 }, { fabgl::VK_DOLLAR,     0, 0x10 }, { fabgl::VK_KP_4, 0, 0x10 },
  { fabgl::VK_5, 0, 0x20 }, { fabgl::VK_PERCENT,    0, 0x20 }, { fabgl::VK_KP_5, 0, 0x20 },
  { fabgl::VK_6, 0, 0x40 }, { fabgl::VK_CARET,      0, 0x40 }, { fabgl::VK_KP_6, 0, 0x40 },
  { fabgl::VK_7, 0, 0x80 }, { fabgl::VK_AMPERSAND,  0, 0x80 }, { fabgl::VK_KP_7, 0, 0x80 },

  // --- Fila 1: 8, 9, -, =, \, [, ], ; (+ símbolos con SHIFT) ---
  { fabgl::VK_8,            1, 0x01 }, { fabgl::VK_ASTERISK,    1, 0x01 }, { fabgl::VK_KP_8, 1, 0x01 },
  { fabgl::VK_9,            1, 0x02 }, { fabgl::VK_LEFTPAREN,   1, 0x02 }, { fabgl::VK_KP_9, 1, 0x02 },
  { fabgl::VK_MINUS,        1, 0x04 }, { fabgl::VK_UNDERSCORE,  1, 0x04 }, { fabgl::VK_KP_MINUS, 1, 0x04 },
  { fabgl::VK_EQUALS,       1, 0x08 }, { fabgl::VK_PLUS,        1, 0x08 },
  { fabgl::VK_BACKSLASH,    1, 0x10 }, { fabgl::VK_VERTICALBAR, 1, 0x10 },
  { fabgl::VK_LEFTBRACKET,  1, 0x20 }, { fabgl::VK_LEFTBRACE,   1, 0x20 },
  { fabgl::VK_RIGHTBRACKET, 1, 0x40 }, { fabgl::VK_RIGHTBRACE,  1, 0x40 },
  { fabgl::VK_SEMICOLON,    1, 0x80 }, { fabgl::VK_COLON,       1, 0x80 }, { fabgl::VK_KP_PLUS, 1, 0x80 }, // numpad + -> ; (mapeo original)

  // --- Fila 2: ', `, ,, ., /, (dead), A, B ---
  { fabgl::VK_QUOTE,       2, 0x01 }, { fabgl::VK_QUOTEDBL, 2, 0x01 },
  { fabgl::VK_GRAVEACCENT, 2, 0x02 }, { fabgl::VK_TILDE,    2, 0x02 },
  { fabgl::VK_COMMA,       2, 0x04 }, { fabgl::VK_LESS,     2, 0x04 },
  { fabgl::VK_PERIOD,      2, 0x08 }, { fabgl::VK_GREATER,  2, 0x08 }, { fabgl::VK_KP_PERIOD, 2, 0x08 }, // numpad . (mapeo original)
  { fabgl::VK_SLASH,       2, 0x10 }, { fabgl::VK_QUESTION, 2, 0x10 }, { fabgl::VK_KP_DIVIDE, 2, 0x10 }, // numpad / (mapeo original)
  // bit 0x20 = dead key -> se descarta intencionadamente (igual que el original)
  { fabgl::VK_A, 2, 0x40 }, { fabgl::VK_a, 2, 0x40 },
  { fabgl::VK_B, 2, 0x80 }, { fabgl::VK_b, 2, 0x80 }, { fabgl::VK_KP_MULTIPLY, 2, 0x80 }, // numpad * -> : (mapeo original)

  // --- Fila 3: C, D, E, F, G, H, I, J ---
  { fabgl::VK_C, 3, 0x01 }, { fabgl::VK_c, 3, 0x01 },
  { fabgl::VK_D, 3, 0x02 }, { fabgl::VK_d, 3, 0x02 },
  { fabgl::VK_E, 3, 0x04 }, { fabgl::VK_e, 3, 0x04 },
  { fabgl::VK_F, 3, 0x08 }, { fabgl::VK_f, 3, 0x08 },
  { fabgl::VK_G, 3, 0x10 }, { fabgl::VK_g, 3, 0x10 },
  { fabgl::VK_H, 3, 0x20 }, { fabgl::VK_h, 3, 0x20 },
  { fabgl::VK_I, 3, 0x40 }, { fabgl::VK_i, 3, 0x40 },
  { fabgl::VK_J, 3, 0x80 }, { fabgl::VK_j, 3, 0x80 },

  // --- Fila 4: K, L, M, N, O, P, Q, R ---
  { fabgl::VK_K, 4, 0x01 }, { fabgl::VK_k, 4, 0x01 },
  { fabgl::VK_L, 4, 0x02 }, { fabgl::VK_l, 4, 0x02 },
  { fabgl::VK_M, 4, 0x04 }, { fabgl::VK_m, 4, 0x04 },
  { fabgl::VK_N, 4, 0x08 }, { fabgl::VK_n, 4, 0x08 },
  { fabgl::VK_O, 4, 0x10 }, { fabgl::VK_o, 4, 0x10 },
  { fabgl::VK_P, 4, 0x20 }, { fabgl::VK_p, 4, 0x20 },
  { fabgl::VK_Q, 4, 0x40 }, { fabgl::VK_q, 4, 0x40 },
  { fabgl::VK_R, 4, 0x80 }, { fabgl::VK_r, 4, 0x80 },

  // --- Fila 5: S, T, U, V, W, X, Y, Z ---
  { fabgl::VK_S, 5, 0x01 }, { fabgl::VK_s, 5, 0x01 },
  { fabgl::VK_T, 5, 0x02 }, { fabgl::VK_t, 5, 0x02 },
  { fabgl::VK_U, 5, 0x04 }, { fabgl::VK_u, 5, 0x04 },
  { fabgl::VK_V, 5, 0x08 }, { fabgl::VK_v, 5, 0x08 },
  { fabgl::VK_W, 5, 0x10 }, { fabgl::VK_w, 5, 0x10 },
  { fabgl::VK_X, 5, 0x20 }, { fabgl::VK_x, 5, 0x20 },
  { fabgl::VK_Y, 5, 0x40 }, { fabgl::VK_y, 5, 0x40 },
  { fabgl::VK_Z, 5, 0x80 }, { fabgl::VK_z, 5, 0x80 },

  // --- Fila 6: SHIFT, CTRL, GRAPH, CAPS, CODE, F1, F2, F3 ---
  { fabgl::VK_LSHIFT, 6, 0x01 }, { fabgl::VK_RSHIFT, 6, 0x01 },
  { fabgl::VK_LCTRL,  6, 0x02 }, { fabgl::VK_RCTRL,  6, 0x02 },
  { fabgl::VK_LALT,     6, 0x04 }, // GRAPH
  { fabgl::VK_CAPSLOCK, 6, 0x08 },
  { fabgl::VK_RALT,     6, 0x10 }, // CODE
  { fabgl::VK_F1, 6, 0x20 },
  { fabgl::VK_F2, 6, 0x40 },
  { fabgl::VK_F3, 6, 0x80 },

  // --- Fila 7: F4, F5, ESC, TAB, STOP, BS, SELECT, RETURN ---
  { fabgl::VK_F4,        7, 0x01 },
  { fabgl::VK_F5,        7, 0x02 },
  { fabgl::VK_ESCAPE,    7, 0x04 },
  { fabgl::VK_TAB,       7, 0x08 },
  { fabgl::VK_PAUSE,     7, 0x10 }, // STOP
  { fabgl::VK_BACKSPACE, 7, 0x20 },
  { fabgl::VK_HOME,      7, 0x40 }, // SELECT
  { fabgl::VK_RETURN,    7, 0x80 },

  // --- Fila 8: SPACE, HOME(CLS), INS, DEL, LEFT, UP, DOWN, RIGHT ---
  { fabgl::VK_SPACE,  8, 0x01 },
  { fabgl::VK_END,    8, 0x02 }, // HOME(CLS)
  { fabgl::VK_INSERT, 8, 0x04 },
  { fabgl::VK_DELETE, 8, 0x08 },
  { fabgl::VK_LEFT,   8, 0x10 },
  { fabgl::VK_UP,     8, 0x20 },
  { fabgl::VK_DOWN,   8, 0x40 },
  { fabgl::VK_RIGHT,  8, 0x80 },
};

static constexpr int KeyMapSize = sizeof(KeyMap) / sizeof(KeyMap[0]);

void ProcessKey(fabgl::VirtualKey vk, bool down) {
  // Tecla de salida (F12): no forma parte de la matriz MSX
  if (down && vk == fabgl::VK_F12) {
    ExitNow = 1;
    return;
  }

  for (int i = 0; i < KeyMapSize; i++) {
    if (KeyMap[i].vk == vk) {
      if (down)
        PressKey(KeyMap[i].row, KeyMap[i].mask);
      else
        ReleaseKey(KeyMap[i].row, KeyMap[i].mask);
      return; // cada virtual key aparece una única vez en la tabla
    }
  }
  // Tecla no mapeada: se ignora silenciosamente (igual que el 'default' original)
}

// [복구] 키보드 업데이트 함수
void UpdateMSXKeyboard() {
  fabgl::Keyboard *kb = PS2Controller.keyboard();
  if(!kb) return;

  while(kb->virtualKeyAvailable()) {
    bool down;
    fabgl::VirtualKey vk = kb->getNextVirtualKey(&down);
    ProcessKey(vk, down);
  }
}

// MSX 초기 색상 테이블 (부팅 시 사용)
fabgl::RGB888 msxColor(uint8_t c) {
  // MSX 표준 RGB 테이블
  static const uint8_t stdPal[16][3] = {
    {0,0,0}, {0,0,0}, {32,192,32}, {96,224,96},
    {32,32,224}, {64,96,224}, {160,32,32}, {64,192,224},
    {224,32,32}, {224,96,96}, {192,192,32}, {192,192,128},
    {32,128,32}, {192,64,160}, {160,160,160}, {224,224,224}
  };
  
  uint8_t idx = c & 0x0F;
  return convertColor(stdPal[idx][0], stdPal[idx][1], stdPal[idx][2]);
}

int InitMachine(void) {
  Serial.println("InitMachine: Starting...");
  
  // 객체 연결 및 포인터 설정
  if (!ensureVga16()) {
    Serial.println("ERROR: VGA16Controller not available!");
    return 0;
  }
  
  // 초기 팔레트 설정 (부팅 직후 화면 색상)
  for(int i = 0; i < 16; i++) {
    vga16->setPaletteItem(i, msxColor(i));
  }

  // 화면 버퍼 할당
  size_t bufSize = MSX_WIDTH * MSX_HEIGHT;
  XBuf = (uint8_t*)ps_malloc(bufSize);
  if(!XBuf) {
    Serial.println("Failed to allocate XBuf");
    return 0;
  }
  memset(XBuf, 0, bufSize);
  
  msxScreen = new fabgl::Bitmap(MSX_WIDTH, MSX_HEIGHT, XBuf, 
                                 fabgl::PixelFormat::Native, false);
  if(!msxScreen) {
    Serial.println("Failed to create msxScreen");
    free(XBuf);
    XBuf = NULL;
    return 0;
  }
  
  Serial.println("InitMachine completed successfully");
  return 1;
}

void TrashMachine(void) {
  if(msxScreen) { delete msxScreen; msxScreen = nullptr; }
  if(XBuf) { free(XBuf); XBuf = NULL; }
}

extern "C" void SetColor(byte N, byte R, byte G, byte B) {
  if(N >= 16) return;
  if(!ensureVga16()) return;
  
  // 변환된 색상 적용
  fabgl::RGB888 color = convertColor(R, G, B);
  vga16->setPaletteItem(N, color);
}

void RefreshScreen(void) {
  if(Canvas && msxScreen) {
    Canvas->drawBitmap(VGA_OFFSET_X, VGA_OFFSET_Y, msxScreen);
  }
}

void DrawSprites(byte Y, byte *LineBuffer) {
  if (SpritesOFF) return;

  int Size = (VDP[1] & 2) ? 16 : 8;
  int Mag  = (VDP[1] & 1); 

  for (int i = 0; i < 32; i++) {
    byte *Attr = SprTab + i * 4;
    int Sy = Attr[0];
    if (Sy == 208) break; 

    if (Sy > 240) Sy -= 256;
    Sy++;

    int Diff = Y - Sy;
    if (Mag) Diff >>= 1;

    if (Diff >= 0 && Diff < Size) {
      int Sx = Attr[1];
      int PatIdx = Attr[2];
      byte Color = Attr[3] & 0x0F;

      if (Attr[3] & 0x80) Sx -= 32;
      if (Color == 0) continue;
      if (Size == 16) PatIdx &= 0xFC;

      byte PatLine = SprGen[PatIdx * 8 + Diff];

      for (int p = 0; p < 8; p++) {
        int DrawX = Sx + (Mag ? p * 2 : p);
        if ((PatLine & 0x80) && (DrawX >= 0) && (DrawX < MSX_WIDTH)) {
          LineBuffer[DrawX] = Color;
          if (Mag && (DrawX + 1 < MSX_WIDTH)) LineBuffer[DrawX + 1] = Color;
        }
        PatLine <<= 1;
      }
      if (Size == 16) {
        PatLine = SprGen[PatIdx * 8 + Diff + 16];
        for (int p = 0; p < 8; p++) {
          int DrawX = Sx + (Mag ? (p + 8) * 2 : (p + 8));
           if ((PatLine & 0x80) && (DrawX >= 0) && (DrawX < MSX_WIDTH)) {
            LineBuffer[DrawX] = Color;
            if (Mag && (DrawX + 1 < MSX_WIDTH)) LineBuffer[DrawX + 1] = Color;
          }
          PatLine <<= 1;
        }
      }
    }
  }
}

void RefreshLine0(byte Y) {
  if(!XBuf || Y >= MSX_HEIGHT) return;
  byte *P = XBuf + Y * MSX_WIDTH;
  byte BG = VDP[7] & 0x0F;
  byte FG = VDP[7] >> 4;
  
  memset(P, BG, 8); 
  P += 8;
  int Row = (Y >> 3) * 40; 
  int Line = Y & 7;
  for (int X = 0; X < 40; X++) {
    int CharCode = ChrTab[Row + X];
    byte Pattern = ChrGen[CharCode * 8 + Line];
    for (int i = 0; i < 6; i++) {
      *P++ = (Pattern & 0x80) ? FG : BG;
      Pattern <<= 1;
    }
  }
  memset(P, BG, 8); 
}

void RefreshLine1(byte Y) {
  if(!XBuf || Y >= MSX_HEIGHT) return;
  byte *P = XBuf + Y * MSX_WIDTH;
  int Row = (Y >> 3) * 32;
  int Line = Y & 7;
  byte BackdropColor = VDP[7] & 0x0F;
  for (int X = 0; X < 32; X++) {
    int CharCode = ChrTab[Row + X];
    byte Pattern = ChrGen[CharCode * 8 + Line];
    byte Color = ColTab[CharCode >> 3]; 
    byte FG = Color >> 4;
    byte BG = Color & 0x0F;
    if(FG == 0) FG = BackdropColor;
    if(BG == 0) BG = BackdropColor;
    for(int i = 0; i < 8; i++) {
      *P++ = (Pattern & 0x80) ? FG : BG;
      Pattern <<= 1;
    }
  }
  DrawSprites(Y, XBuf + Y * MSX_WIDTH);
}

void RefreshLine2(byte Y) {
  if(!XBuf || Y >= MSX_HEIGHT) return;
  byte *P = XBuf + Y * MSX_WIDTH;
  int Zone = Y / 64; 
  int Row  = (Y >> 3) * 32; 
  int Line = Y & 7;         
  byte BackdropColor = VDP[7] & 0x0F;
  for (int X = 0; X < 32; X++) {
    int CharCode = ChrTab[Row + X] + (Zone * 256);
    byte Pattern = ChrGen[CharCode * 8 + Line];
    byte Color   = ColTab[CharCode * 8 + Line];
    byte FG = Color >> 4;   
    byte BG = Color & 0x0F; 
    if(FG == 0) FG = BackdropColor;
    if(BG == 0) BG = BackdropColor;
    for(int i=0; i<8; i++) {
      *P++ = (Pattern & 0x80) ? FG : BG;
      Pattern <<= 1;
    }
  }
  DrawSprites(Y, XBuf + Y * MSX_WIDTH);
}

void ClearLine(byte Y) {
  if(!XBuf || Y >= MSX_HEIGHT) return;
  byte *P = XBuf + Y * MSX_WIDTH;
  byte BG = VDP[7] & 0x0F;
  memset(P, BG, MSX_WIDTH);
}

void RefreshLineTx80(byte Y) { ClearLine(Y); }
void RefreshLine3(byte Y) { ClearLine(Y); }
void RefreshLine4(byte Y) { ClearLine(Y); }
void RefreshLine5(byte Y) { ClearLine(Y); }
void RefreshLine6(byte Y) { ClearLine(Y); }
void RefreshLine7(byte Y) { ClearLine(Y); }
void RefreshLine8(byte Y) { ClearLine(Y); }
void RefreshLine10(byte Y) { ClearLine(Y); }
void RefreshLine12(byte Y) { ClearLine(Y); }

void Keyboard(void) { UpdateMSXKeyboard(); }
unsigned int Joystick(void) {
  unsigned int status = 0xFFFF; 
  fabgl::Keyboard *kb = PS2Controller.keyboard();
  if (!kb) return status;
  if (kb->isVKDown(fabgl::VK_UP))    status &= ~0x0001; 
  if (kb->isVKDown(fabgl::VK_DOWN))  status &= ~0x0002; 
  if (kb->isVKDown(fabgl::VK_LEFT))  status &= ~0x0004; 
  if (kb->isVKDown(fabgl::VK_RIGHT)) status &= ~0x0008; 
  if (kb->isVKDown(fabgl::VK_SPACE)) status &= ~0x0010; 
  if (kb->isVKDown(fabgl::VK_M))     status &= ~0x0020; 
  return status;
}
unsigned int Mouse(byte N) { return 0; }
void PutImage(void) { RefreshScreen(); }

// SD 카드 파일 I/O
extern "C" {
  void* arduino_fopen(const char* filename, const char* mode) {
    if(!filename) return NULL;
    String path = String(filename);
    if(!path.startsWith("/")) path = "/" + path;
    File tempFile = SD.open(path, FILE_READ);
    if(!tempFile) return NULL;
    File* fptr = new File(tempFile);
    return (void*)fptr;
  }
  int arduino_fclose(void* stream) {
    if(!stream) return 0;
    File* f = (File*)stream;
    f->close();
    delete f;
    return 0;
  }
  size_t arduino_fread(void* ptr, size_t size, size_t nmemb, void* stream) {
    if(!stream || !ptr || size == 0 || nmemb == 0) return 0;
    File* f = (File*)stream;
    if(!f->available()) return 0;
    size_t bytesRead = f->read((uint8_t*)ptr, size * nmemb);
    return bytesRead / size;
  }
  size_t arduino_fwrite(const void* ptr, size_t size, size_t nmemb, void* stream) {
    if(!stream || !ptr || size == 0 || nmemb == 0) return 0;
    File* f = (File*)stream;
    size_t bytesWritten = f->write((const uint8_t*)ptr, size * nmemb);
    return bytesWritten / size;
  }
  int arduino_fseek(void* stream, long offset, int whence) {
    if(!stream) return -1;
    File* f = (File*)stream;
    if(whence == SEEK_SET) f->seek(offset);
    else if(whence == SEEK_END) f->seek(f->size() + offset);
    else if(whence == SEEK_CUR) f->seek(f->position() + offset);
    return 0;
  }
  long arduino_ftell(void* stream) {
    if(!stream) return -1;
    File* f = (File*)stream;
    return f->position();
  }
  void arduino_rewind(void* stream) {
    if(!stream) return;
    File* f = (File*)stream;
    f->seek(0);
  }
  int arduino_fgetc(void* stream) {
    if(!stream) return -1;
    File* f = (File*)stream;
    if(!f->available()) return -1;
    return f->read();
  }
  int arduino_feof(void* stream) {
    if(!stream) return 1;
    File* f = (File*)stream;
    return !f->available();
  }
}