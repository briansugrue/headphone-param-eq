/* 5-band parametric EQ (biquad cascade) for the STM32F407 audio pass-through.
 *
 * - Filter designs: RBJ "Audio EQ Cookbook" peaking and shelving filters, plus the
 *   2nd-order low pass from the PC prototype (zeros at -1, pole radius r).
 * - Structure: Transposed Direct Form II in single-precision float, one state set per
 *   channel. This is much better behaved at low frequencies than Direct Form I.
 * - Headroom: the combined frequency response is evaluated at start-up and the input
 *   is automatically turned down by its peak gain, so boosts cannot overload the
 *   16-bit output for steady signals. The output is still clamped as a backstop.
 * - User button (PA0) toggles bypass. LED PD13 (orange) is on while the EQ is active.
 */

/* The project's Debug configuration builds with -O0, which makes this code several
   times slower. Force optimisation for this file only. */
#pragma GCC optimize ("O2")

#include "eq.h"
#include "main.h"
#include <math.h>

/* ======================= EDIT YOUR EQ HERE AND REFLASH ======================= */

typedef enum
{
  EQ_PEAK,        /* bell: boost/cut around fc.   q = Q (width)                  */
  EQ_LOWSHELF,    /* boost/cut below fc.          q = shelf Q (0.71 = gentle)    */
  EQ_HIGHSHELF,   /* boost/cut above fc.          q = shelf Q (0.71 = gentle)    */
  EQ_LOWPASS_R    /* PC-prototype low pass.       q = pole radius r (0 < r < 1), */
                  /*                              gainDb is ignored              */
} EqType;

typedef struct
{
  EqType type;
  float  fc;       /* centre / corner frequency in Hz                            */
  float  gainDb;   /* boost (+) or cut (-) in dB; 0 on peak/shelf = band unused  */
  float  q;        /* see above                                                  */
} EqBand;

#define NUM_BANDS 5U

static const EqBand bands[NUM_BANDS] =
{
  /* type          fc [Hz]  gain [dB]  Q   */
  { EQ_LOWSHELF,    450.0f,   -24.0f,  0.71f },   /* band 1: kills bass (acts as high-pass) */
  { EQ_PEAK,        200.0f,   -12.0f,  0.80f },   /* band 2: extra low-end removal          */
  { EQ_PEAK,       1500.0f,    +9.0f,  1.80f },   /* band 3: nasal/honky midrange boost     */
  { EQ_PEAK,       4500.0f,   -12.0f,  0.80f },   /* band 4: extra top-end removal          */
  { EQ_HIGHSHELF,  3200.0f,   -24.0f,  0.71f },   /* band 5: kills treble (acts as low-pass)*/
};

#define HEADROOM_MARGIN_DB  0.5f   /* extra safety below the computed peak gain */

/* ============================================================================ */

#define PI_D              3.14159265358979323846
#define BTN_STABLE_CALLS  5U       /* ~13 ms at one call per 2.67 ms            */
#define RESP_POINTS       400U     /* frequencies used to find the peak gain    */

typedef struct { float b0, b1, b2, a1, a2; } BiquadCoef;   /* a0 normalised to 1 */
typedef struct { float z1, z2; }             BiquadState;

static BiquadCoef  coef[NUM_BANDS];                 /* active stages, packed from index 0 */
static BiquadState state[NUM_BANDS][2];             /* [stage][0 = left, 1 = right]       */
static uint32_t    numActive;
static float       preGain = 1.0f;

static volatile uint8_t eqBypass;                   /* visible in Live Expressions */
volatile float          eqPreGainDb;                /* visible in Live Expressions */
volatile uint32_t       dbgEqCyclesMax;             /* longest EQ_Process call in CPU cycles.
                                                       Budget: 2.67 ms = 384000 cycles; keep it
                                                       under about 100000 */
static uint8_t btnCount;
static uint8_t btnHeld;

/* Coefficients in double, normalised (a0 = 1) */
typedef struct { double b0, b1, b2, a1, a2; } CoefD;

static int design_band(CoefD *o, const EqBand *bd, double fs)
{
  double fc = (double)bd->fc;
  if (fc < 10.0)        fc = 10.0;
  if (fc > 0.45 * fs)   fc = 0.45 * fs;           /* keep well below Nyquist */

  double w0 = 2.0 * PI_D * fc / fs;
  double cw = cos(w0);
  double sw = sin(w0);

  double b0, b1, b2, a0, a1, a2;

  if (bd->type == EQ_LOWPASS_R)
  {
    double r = (double)bd->q;
    if (r <= 0.0 || r >= 1.0) return 0;
    a0 = 1.0;
    a1 = -2.0 * r * cw;
    a2 = r * r;
    double g = 4.0 / (1.0 + a1 + a2);              /* normalise DC gain to 1 */
    b0 = 1.0 / g;  b1 = 2.0 / g;  b2 = 1.0 / g;
  }
  else
  {
    if (bd->gainDb == 0.0f) return 0;              /* flat band: skip it */

    double q = ((double)bd->q > 0.05) ? (double)bd->q : 0.05;
    double A = pow(10.0, (double)bd->gainDb / 40.0);

    if (bd->type == EQ_PEAK)
    {
      double alpha = sw / (2.0 * q);
      b0 = 1.0 + alpha * A;
      b1 = -2.0 * cw;
      b2 = 1.0 - alpha * A;
      a0 = 1.0 + alpha / A;
      a1 = -2.0 * cw;
      a2 = 1.0 - alpha / A;
    }
    else
    {
      double alpha = sw / (2.0 * q);
      double tsa   = 2.0 * sqrt(A) * alpha;

      if (bd->type == EQ_LOWSHELF)
      {
        b0 =        A * ((A + 1.0) - (A - 1.0) * cw + tsa);
        b1 =  2.0 * A * ((A - 1.0) - (A + 1.0) * cw);
        b2 =        A * ((A + 1.0) - (A - 1.0) * cw - tsa);
        a0 =             (A + 1.0) + (A - 1.0) * cw + tsa;
        a1 = -2.0 *     ((A - 1.0) + (A + 1.0) * cw);
        a2 =             (A + 1.0) + (A - 1.0) * cw - tsa;
      }
      else /* EQ_HIGHSHELF */
      {
        b0 =        A * ((A + 1.0) + (A - 1.0) * cw + tsa);
        b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cw);
        b2 =        A * ((A + 1.0) + (A - 1.0) * cw - tsa);
        a0 =             (A + 1.0) - (A - 1.0) * cw + tsa;
        a1 =  2.0 *     ((A - 1.0) - (A + 1.0) * cw);
        a2 =             (A + 1.0) - (A - 1.0) * cw - tsa;
      }
    }
  }

  o->b0 = b0 / a0;  o->b1 = b1 / a0;  o->b2 = b2 / a0;
  o->a1 = a1 / a0;  o->a2 = a2 / a0;
  return 1;
}

/* |H(e^jw)|^2 of one stage */
static double stage_mag2(const CoefD *c, double w)
{
  double c1 = cos(w),       s1 = sin(w);
  double c2 = cos(2.0 * w), s2 = sin(2.0 * w);

  double nr = c->b0 + c->b1 * c1 + c->b2 * c2;
  double ni =       -(c->b1 * s1 + c->b2 * s2);
  double dr = 1.0   + c->a1 * c1 + c->a2 * c2;
  double di =       -(c->a1 * s1 + c->a2 * s2);

  return (nr * nr + ni * ni) / (dr * dr + di * di);
}

static void clear_state(void)
{
  for (uint32_t s = 0; s < NUM_BANDS; s++)
  {
    state[s][0] = (BiquadState){0};
    state[s][1] = (BiquadState){0};
  }
}

static void set_led(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, eqBypass ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void EQ_Init(float fs_f)
{
  double fs = (double)fs_f;
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Cycle counter, used to measure how long EQ_Process takes */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  /* User button B1 on PA0 (external pull-down on the board, active high) */
  g.Pin  = GPIO_PIN_0;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);

  /* Orange LED on PD13 */
  g.Pin   = GPIO_PIN_13;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &g);

  /* Design all bands, keeping only the ones that do something */
  CoefD dc[NUM_BANDS];
  numActive = 0U;
  for (uint32_t i = 0; i < NUM_BANDS; i++)
  {
    CoefD d;
    if (design_band(&d, &bands[i], fs))
    {
      dc[numActive]   = d;
      coef[numActive] = (BiquadCoef){ (float)d.b0, (float)d.b1, (float)d.b2,
                                      (float)d.a1, (float)d.a2 };
      numActive++;
    }
  }

  /* Find the peak of the combined response (20 Hz .. 20 kHz) and turn the input
     down by that much, plus a small margin */
  double peak2 = 1.0;
  for (uint32_t k = 0; k < RESP_POINTS; k++)
  {
    double f = 20.0 * pow(1000.0, (double)k / (double)(RESP_POINTS - 1U));   /* 20 Hz .. 20 kHz */
    if (f > 0.49 * fs) break;
    double w = 2.0 * PI_D * f / fs;

    double m2 = 1.0;
    for (uint32_t s = 0; s < numActive; s++) m2 *= stage_mag2(&dc[s], w);
    if (m2 > peak2) peak2 = m2;
  }
  double peakDb = 10.0 * log10(peak2);                       /* >= 0 dB */
  eqPreGainDb   = -(float)peakDb - HEADROOM_MARGIN_DB;
  preGain       = (float)pow(10.0, (double)eqPreGainDb / 20.0);

  clear_state();
  btnCount = 0U;
  btnHeld  = 0U;
  set_led();
}

/* Called once per refill (every 2.67 ms): debounced toggle on button press */
static void poll_button(void)
{
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
  {
    if (btnCount < BTN_STABLE_CALLS)
    {
      btnCount++;
    }
    else if (!btnHeld)
    {
      btnHeld   = 1U;
      eqBypass ^= 1U;
      clear_state();              /* don't resume with stale filter memory */
      set_led();
    }
  }
  else
  {
    btnCount = 0U;
    btnHeld  = 0U;
  }
}

void EQ_Process(int16_t *buf, uint32_t frames)
{
  poll_button();

  if (eqBypass) return;

  const float scale = preGain / 32768.0f;
  const uint32_t t0 = DWT->CYCCNT;

  for (uint32_t i = 0; i < frames; i++)
  {
    for (uint32_t ch = 0; ch < 2U; ch++)
    {
      float x = (float)buf[2U * i + ch] * scale;

      for (uint32_t s = 0; s < numActive; s++)
      {
        const BiquadCoef *c  = &coef[s];
        BiquadState      *st = &state[s][ch];

        /* Transposed Direct Form II */
        float y = c->b0 * x + st->z1;
        st->z1  = c->b1 * x - c->a1 * y + st->z2;
        st->z2  = c->b2 * x - c->a2 * y;
        x = y;
      }

      /* Back to 16 bit with rounding, clamped so overs don't wrap around */
      float v = x * 32768.0f;
      if (v >  32767.0f) v =  32767.0f;
      if (v < -32768.0f) v = -32768.0f;
      buf[2U * i + ch] = (int16_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
    }
  }

  uint32_t used = DWT->CYCCNT - t0;
  if (used > dbgEqCyclesMax) dbgEqCyclesMax = used;
}
