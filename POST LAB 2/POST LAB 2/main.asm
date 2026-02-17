/*
* Laboratorio2.asm                                                     
* Creado: 10/02/2026                                                  
* Autor : Eddy Gómez                                                  
* Descripción:
*   Se implementa un contador de 4 bits en los LEDs D2–D5 utilizando el Timer0 en modo normal y sin interrupciones:
*   el Timer0 genera un “tick” interno de ~100ms por detección de overflow y, al acumular 10 ticks, el contador incrementa cada ~1s.
*   Un segundo contador controlado con botones en A0 (+) y A1 (–) con anti-rebote se muestra en un display de 7 segmentos ánodo común
*   conectado de D6 a D12, el cual inicia apagado y solo se actualiza al presionar un botón. Si el display está en 0, el contador de LEDs
*   se congela y permanece apagado; cuando el valor de los LEDs coincide con el del display, el valor queda visible durante 1 segundo completo
*   y luego se reinicia el conteo, alternando (toggle) el estado del indicador en D13 para poder variar su periodo usando los botones.
*/

/**************/                                                     
.include "M328PDEF.inc"                                              // Se cargan definiciones del ATmega328P (registros, bits y nombres útiles)

/* Registros */                                                      // Bloque de registros: cada .def asigna un registro AVR a un nombre del programa
.def contador   = r16                                                // contador: valor de 4 bits que se muestra en LEDs D2..D5 (0..15)
.def aux        = r17                                                // aux: registro temporal para operaciones internas (máscaras, shifts, etc.)
.def aux2       = r18                                                // aux2: temporal para leer/armar puertos sin destruir bits ajenos
.def contador2  = r19                                                // contador2: valor 0..15 ajustado por botones y mostrado en el 7 segmentos

.def t0ovf      = r20                                                // t0ovf: cantidad de overflows restantes para completar ~100ms
.def segpat     = r21                                                // segpat: patrón de segmentos a..g obtenido de la tabla en FLASH
.def tmp        = r22                                                // tmp: bandera de tick (tmp=1 cuando ya ocurrió el tick ~100ms)
.def tickst     = r23                                                // tickst: indica si el “ciclo de conteo” de ~100ms está activo

.def sub100     = r24                                                // sub100: acumula ticks de 100ms (0..9); cuando llega a 10 forma ~1 segundo
.def ledst      = r25                                                // ledst: estado lógico del indicador en D13 (toggle 0/1)

.def holdst     = r26                                                // holdst: marca modo HOLD (1=se retiene el valor 1s antes del reset)
.def hold100    = r27                                                // hold100: acumula ticks de 100ms durante HOLD (0..9 = ~1s)

.dseg                                                               // Segmento de datos SRAM (reservas en RAM, si se necesitaran variables)
.org SRAM_START                                                     // Punto base de SRAM (aquí se podría reservar memoria; en este programa se usan registros)

/**************/                                                     
.cseg                                                               // Segmento de código en FLASH
.org 0x0000                                                         // Vector de reset (dirección 0)
    rjmp RESET                                                      // Salto al inicio real del programa para no ejecutar basura

/**************/                                                     
RESET:                                                              // Punto de arranque después del reset
    LDI     R16, LOW(RAMEND)                                        // Byte bajo del final de RAM para inicializar el stack
    OUT     SPL, R16                                                // Se configura SPL
    LDI     R16, HIGH(RAMEND)                                       // Byte alto del final de RAM
    OUT     SPH, R16                                                // Se configura SPH

table7seg:                                                          // Tabla en FLASH con patrones para display ánodo común (0 enciende, 1 apaga)
    .db 0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78
    .db 0x00, 0x10, 0x08, 0x03, 0x46, 0x21, 0x06, 0x0E
;          0     1     2     3     4     5     6     7
;          8     9     A     b     C     d     E     F
; La tabla se recorre con Z y LPM para obtener el patrón del dígito/hex correspondiente

/**************/                                                     
SETUP:                                                              // Configuración inicial del micro (puertos, timer y estados)
    clr r1                                                          // r1 se mantiene en 0 por convención (muchas rutinas lo asumen)

    clr contador                                                    // contador de LEDs inicia en 0 (binario 0000)
    clr contador2                                                   // contador2 inicia en 0 (display apagado hasta presionar botones)
    clr tickst                                                      // el tick de ~100ms aún no está iniciado
    clr sub100                                                      // acumulador de 100ms inicia en 0
    clr ledst                                                       // indicador lógico de D13 inicia en 0 (apagado)

    clr holdst                                                      // no se está reteniendo el valor (HOLD desactivado)
    clr hold100                                                     // contador de retención (HOLD) inicia en 0

    ; PD2..PD5 = LEDs (contador binario)                             // Parte baja del “panel” de LEDs
    ; PD6..PD7 = segmentos a,b (display)                             // Parte alta del display (a y b)
    ldi aux, 0b11111100                                             // Se configuran PD2..PD7 como salida; PD0..PD1 quedan como entrada (UART)
    out DDRD, aux                                                   // DDRD define direcciones de PORTD

    ; PB0..PB4 = segmentos c..g (display)                            // Segmentos restantes c..g
    ; PB5      = D13 indicador/alarma                                // LED indicador en D13
    ldi aux, 0b00111111                                             // PB0..PB5 como salida; PB6..PB7 libres para cristal
    out DDRB, aux                                                   // DDRB define direcciones de PORTB

    ; Display apagado al inicio (ánodo común: 1 apaga)               // Se fuerza a “apagado” antes de cualquier actualización
    sbi PORTD, 6                                                    // PD6=1 -> segmento a apagado
    sbi PORTD, 7                                                    // PD7=1 -> segmento b apagado
    sbi PORTB, 0                                                    // PB0=1 -> segmento c apagado
    sbi PORTB, 1                                                    // PB1=1 -> segmento d apagado
    sbi PORTB, 2                                                    // PB2=1 -> segmento e apagado
    sbi PORTB, 3                                                    // PB3=1 -> segmento f apagado
    sbi PORTB, 4                                                    // PB4=1 -> segmento g apagado

    ; Botones A0(PC0) y A1(PC1) con pull-up                          // Entrada con pull-up: suelto=1, presionado=0
    cbi DDRC, 0                                                     // PC0 como entrada (+)
    cbi DDRC, 1                                                     // PC1 como entrada (-)
    sbi PORTC, 0                                                    // pull-up en PC0
    sbi PORTC, 1                                                    // pull-up en PC1

    ; Timer0: NORMAL, prescaler 1024                                 // Base del tick: modo normal y conteo con prescaler
    ldi aux, 0x00                                                   // WGM00=WGM01=0 -> modo normal
    out TCCR0A, aux
    ldi aux, (1<<CS02) | (1<<CS00)                                  // prescaler=1024 (CS02=1 y CS00=1)
    out TCCR0B, aux                                                 // el timer queda corriendo

    ; D13 apagado al inicio                                          // Indicador físico apagado desde el inicio
    cbi PORTB, 5                                                    // PB5=0

    rcall Mostrar                                                   // Los LEDs D2..D5 muestran 0000 al arrancar
    ; NO Mostrar7SEG al inicio                                       // El display debe permanecer apagado hasta el primer botón

/**************/                                                     
MAIN_LOOP:                                                          
    call ContadorBoton                                              // Lectura de botones con anti-rebote (actualiza contador2 y el display)
    rcall Tick100ms_Timer0                                          // Generación del tick ~100ms en tmp (sin interrupciones)

    ; ===== Si display está en 0, NO cuenta y LEDs apagados =====
    tst contador2                                                   // Se verifica si el valor del display es 0
    brne DISP_NOZERO                                                // Si NO es 0, se continúa con la lógica normal

    ; display=0 -> congelar todo el conteo de LEDs                   // Se evita conteo y se fuerza visual 0000 en LEDs
    clr contador                                                    // contador se fuerza a 0
    clr sub100                                                      // se reinicia acumulador de 1s para no arrastrar tiempo guardado
    clr holdst                                                      // se cancela HOLD por si estaba activo
    clr hold100                                                     // se reinicia el contador de HOLD
    rcall Mostrar                                                   // LEDs quedan apagados (0000)
    rjmp MAIN_LOOP                                                  // Se regresa al inicio sin avanzar tiempo

DISP_NOZERO:                                                        // Etiqueta: se permite conteo porque contador2 != 0

    ; ===== Si no hay tick, no se avanza tiempo =====                // tmp solo vale 1 cuando se completó ~100ms
    tst tmp                                                         // Se revisa tmp: 0 = sin tick, 1 = tick listo
    breq NO_TICK                                                    // Sin tick, no se modifica sub100/hold100/contador

    ; ===== Si estamos en HOLD: esperar 1 segundo antes de reiniciar =====
    tst holdst                                                      // Se revisa si HOLD está activo (1)
    breq NORMAL_COUNT                                               // Si HOLD=0, se realiza conteo normal de segundos

    ; HOLD activo: juntar 10 ticks de 100ms -> 1s                    // Durante HOLD el valor en LEDs se mantiene fijo
    inc hold100                                                     // Se incrementa hold100 por cada tick de ~100ms
    cpi hold100, 10                                                 // Se compara si ya se juntaron 10 ticks (~1s)
    brlo NO_TICK                                                    // Si aún no llega a 10, se sigue reteniendo el valor actual

    ; ya pasó 1s en HOLD -> ahora sí reinicia y hace toggle          // Se ejecuta la acción del postlab después de 1 segundo visible
    clr hold100                                                     // Se limpia el acumulador de HOLD
    clr holdst                                                      // Se sale del modo HOLD

    clr contador                                                    // contador vuelve a 0 después de sostener el valor un segundo completo
    rcall Mostrar                                                   // LEDs reflejan el reinicio (0000)

    ldi aux, 1                                                      // Se carga 1 para usarse como máscara de toggle
    eor ledst, aux                                                  // ledst cambia de 0->1 o 1->0 con XOR

    tst ledst                                                       // Se revisa el nuevo estado lógico del indicador
    breq IND_OFF                                                    // Si ledst=0, toca apagar; si ledst!=0, toca encender

IND_ON:
    sbi PORTB, 5                                                    // D13 ON (PB5=1)
    rjmp NO_TICK                                                    // Se regresa al loop sin contar extra en este tick

IND_OFF:
    cbi PORTB, 5                                                    // D13 OFF (PB5=0)
    rjmp NO_TICK                                                    // Se regresa al loop

NORMAL_COUNT:                                                       // Conteo normal (sin HOLD): forma 1 segundo con 10 ticks de 100ms
    ; ===== Conteo normal: 10 ticks de 100ms -> 1 segundo =====
    inc sub100                                                      // Se suma un tick (~100ms) al acumulador
    cpi sub100, 10                                                  // Se verifica si ya se juntó ~1s completo
    brlo NO_TICK                                                    // Si aún no, se espera al siguiente tick

    clr sub100                                                      // Se reinicia el acumulador porque ya se completó un segundo

    inc contador                                                    // contador se incrementa 1 unidad por segundo
    andi contador, 0x0F                                             // Se limita a 4 bits (0..15) para no salir del rango
    rcall Mostrar                                                   // Se actualizan LEDs D2..D5 con el valor nuevo

    ; ===== REGLA 2: si contador == contador2, entrar en HOLD =====
    rcall PeriodoCheckHold                                          // Si hay match, se activa HOLD para sostener el valor 1 segundo

NO_TICK:                                                            // Etiqueta: salida común cuando no toca avanzar
    rjmp MAIN_LOOP                                                  // Loop infinito

/**************/                                                     
/* Timer0 NORMAL no bloqueante (~100ms)
   F_CPU=16MHz, prescaler=1024 => tick=64us
   1 overflow parcial TCNT0=229 => 27 ticks = 1.728ms
   + 6 overflows completos      => 98.304ms
   total ? 100.032ms
   tmp=1 cuando ya pasaron ~100ms
*/
Tick100ms_Timer0:                                                   // Rutina: produce tmp=1 cada ~100ms sin detener el programa
    clr tmp                                                         // tmp inicia en 0 para asumir “sin tick” hasta confirmarlo

    tst tickst                                                      // Se revisa si el conteo de ~100ms ya está activo
    brne T0_CHECK                                                   // Si tickst!=0, toca revisar overflow

    ldi t0ovf, 6                                                    // Se cargan 6 overflows completos como parte del tiempo total
    ldi aux, 229                                                    // Se inicia TCNT0 en 229 para un primer overflow parcial
    out TCNT0, aux                                                  // TCNT0 queda cargado (reduce el primer intervalo)

    ldi aux, (1<<TOV0)                                              // Se prepara el bit para limpiar la bandera TOV0
    out TIFR0, aux                                                  // Se limpia TOV0 por si venía arrastrada

    ldi tickst, 1                                                   // Se marca el ciclo como activo
    ret                                                             // Se sale sin tick en este llamado

T0_CHECK:                                                           // Revisión del overflow
    in  aux2, TIFR0                                                 // Se leen banderas del Timer0
    sbrs aux2, TOV0                                                 // Si TOV0=0, se salta la siguiente instrucción y se retorna
    ret                                                             // Sin overflow aún -> tmp queda en 0

    ldi aux, (1<<TOV0)                                              // Se prepara limpieza de bandera
    out TIFR0, aux                                                  // Se limpia overflow para medir el siguiente

    clr aux                                                         // Para overflows completos siguientes, TCNT0 arranca en 0
    out TCNT0, aux                                                  // TCNT0=0

    dec t0ovf                                                       // Se reduce la cuenta de overflows pendientes
    brne T0_NOTYET                                                  // Si aún faltan overflows, no hay tick todavía

    clr tickst                                                      // Se marca fin del ciclo ~100ms
    ldi tmp, 1                                                      // tmp=1 indica tick listo
    ret                                                             // Se regresa con tick

T0_NOTYET:
    ret                                                             // Se regresa sin tick

/**************/                                                     
/* PostLab HOLD:
   - Si contador == contador2:
       * No se reinicia inmediatamente
       * Se activa HOLD (holdst=1) para retener el valor visible 1 segundo completo
*/
PeriodoCheckHold:                                                   // Rutina: detecta coincidencia y activa el modo de retención
    mov aux, contador                                               // Se copia contador para comparar sin tocar el original
    andi aux, 0x0F                                                  // Se asegura rango 4 bits

    mov aux2, contador2                                             // Se copia contador2 (valor objetivo del periodo)
    cp  aux, aux2                                                   // Se compara contador vs contador2
    brne PCH_NO                                                     // Si no coinciden, se sale sin cambios

    ldi holdst, 1                                                   // Se activa HOLD para sostener el valor antes del reset
    clr hold100                                                     // hold100 inicia en 0 para contar 10 ticks de 100ms
    ret                                                             // Se regresa con HOLD activado

PCH_NO:
    ret                                                             // Sin match, no se activa HOLD

/**************/                                                     
/* Botones: A0=PC0 suma, A1=PC1 resta (pull-up)
   - Presionado = 0 (por pull-up)
   - Se usa Delay20 como anti-rebote
   - Se espera a soltar el botón para no contar repetido
*/
ContadorBoton:                                                      // Rutina: maneja + y - con anti-rebote y actualización del display
    sbis PINC, 0                                                    // Si PC0=1 (no presionado), salta la siguiente instrucción
    rjmp MasB                                                       // Si PC0=0, se detecta botón +

    sbis PINC, 1                                                    // Si PC1=1 (no presionado), salta la siguiente
    rjmp MenosB                                                     // Si PC1=0, se detecta botón -

    ret                                                             // Sin botones, no se modifica contador2

MasB:                                                               // Manejo de botón +
    rcall Delay20                                                   // Retardo para filtrar rebote
    sbis PINC, 0                                                    // Se revisa otra vez: si ya se soltó, se descarta
    rjmp MasB_OK                                                    // Si sigue presionado, pulsación válida
    ret                                                             // Rebote -> salida

MasB_OK:
    inc contador2                                                   // contador2 incrementa 1
    cpi contador2, 16                                               // Se revisa si llegó a 16 (sale de 4 bits)
    brlo MasB_Show                                                  // Si es <16, queda en 0..15
    clr contador2                                                   // Si llegó a 16, se envuelve a 0
MasB_Show:
    rcall Mostrar7SEG                                               // Se actualiza el display (aquí deja de estar apagado)

SoltarMasB:                                                         // Espera a soltar para evitar múltiples conteos por mantener presionado
    sbis PINC, 0                                                    // Si PC0=1 (ya soltó), salta
    rjmp SoltarMasB                                                 // Si sigue en 0, se mantiene esperando
    rcall Delay20                                                   // Antirebote al soltar
    ret                                                             // Regreso al loop

MenosB:                                                             // Manejo de botón -
    rcall Delay20                                                   // Antirebote inicial
    sbis PINC, 1                                                    // Revisión posterior al delay
    rjmp MenosB_OK                                                  // Si sigue presionado, pulsación válida
    ret                                                             // Rebote -> salida

MenosB_OK:
    tst contador2                                                   // Se revisa si contador2 está en 0
    brne MenosB_Dec                                                 // Si no es 0, decremento normal
    ldi contador2, 15                                               // Si era 0, se envuelve a 15
    rjmp MenosB_Show                                                // Se salta al mostrado

MenosB_Dec:
    dec contador2                                                   // Decremento normal

MenosB_Show:
    rcall Mostrar7SEG                                               // Se actualiza el display con el nuevo valor

SoltarMenosB:                                                       // Espera a soltar botón -
    sbis PINC, 1                                                    // Si PC1=1, ya se soltó
    rjmp SoltarMenosB                                               // Si sigue presionado, espera
    rcall Delay20                                                   // Antirebote al soltar
    ret                                                             // Regresa al loop

/**************/                                                     
/* LEDs Timer0 en PD2–PD5
   - Se colocan los 4 bits de contador en PD2..PD5 sin tocar PD0..PD1 ni PD6..PD7
*/
Mostrar:                                                            // Rutina: escribe el valor de contador sobre PD2..PD5
    mov aux, contador                                               // Copia contador a aux para operar sin perderlo
    andi aux, 0x0F                                                  // Se limita a 4 bits
    lsl aux                                                         // Se desplaza 2 posiciones para alinear con PD2
    lsl aux

    in  aux2, PORTD                                                 // Se lee PORTD actual para preservar otros bits
    andi aux2, 0xC3                                                 // Se limpian PD2..PD5 (1100 0011 conserva PD0..1 y PD6..7)
    or  aux2, aux                                                   // Se insertan los bits nuevos en PD2..PD5
    out PORTD, aux2                                                 // Se escribe PORTD actualizado
    ret                                                             // Fin de rutina

/**************/                                                     
/* 7 segmentos ANODO común:
   a=PD6, b=PD7, c=PB0, d=PB1, e=PB2, f=PB3, g=PB4
   - Se toma contador2 como índice (0..15), se busca el patrón en table7seg (FLASH)
   - Se divide el patrón: a,b van a PD6..PD7 y c..g van a PB0..PB4
   - PB5 (D13) se preserva para no afectar el indicador
*/
Mostrar7SEG:                                                        // Rutina: muestra contador2 en el display sin dañar LEDs ni el indicador
    push aux                                                        // Se guardan temporales porque aquí se reutilizan registros
    push aux2
    push segpat
    push tmp

    mov aux, contador2                                              // aux = índice a mostrar (0..15)

    ldi ZH, high(table7seg<<1)                                      // Z apunta al inicio de la tabla en FLASH (dirección *2)
    ldi ZL, low(table7seg<<1)

TAB_ADV:                                                            // Avance en tabla: se corre Z hasta el índice deseado
    tst aux                                                         // Si aux=0, ya está en la entrada correcta
    breq TAB_RD                                                     // Salto a lectura cuando se llega
    adiw Z, 1                                                       // Z avanza 1 byte
    dec aux                                                         // aux baja hasta 0
    rjmp TAB_ADV                                                    // Repetición hasta llegar

TAB_RD:
    lpm segpat, Z                                                   // Se lee el patrón desde FLASH hacia segpat
    andi segpat, 0x7F                                               // Se asegura usar solo 7 bits (a..g)

    ; a,b -> PD6,PD7                                                // Se escriben segmentos a y b en PORTD sin tocar PD0..PD5
    in  aux2, PORTD                                                 // Se lee PORTD actual
    andi aux2, 0b00111111                                           // Se limpian PD6..PD7 preservando PD0..PD5

    mov tmp, segpat                                                 // tmp copia patrón para extraer a,b
    andi tmp, 0b00000011                                            // Se aíslan bits a y b
    lsl tmp                                                         // Se suben a la posición 6..7
    lsl tmp
    lsl tmp
    lsl tmp
    lsl tmp
    lsl tmp

    or  aux2, tmp                                                   // Se insertan a,b en PD6..PD7
    out PORTD, aux2                                                 // Se escribe PORTD

    ; c..g -> PB0..PB4 (preserva PB5)                               // Se escriben segmentos c..g sin cambiar el indicador en PB5
    in  aux2, PORTB                                                 // Se lee PORTB actual
    andi aux2, 0b11100000                                           // Se limpian PB0..PB4 preservando PB5..PB7

    mov tmp, segpat                                                 // tmp toma patrón para extraer c..g
    lsr tmp                                                         // Se quita a
    lsr tmp                                                         // Se quita b (ahora c..g quedan en bits 0..4)
    andi tmp, 0b00011111                                            // Se asegura el rango de 5 bits

    or  aux2, tmp                                                   // Se insertan c..g en PB0..PB4
    out PORTB, aux2                                                 // Se escribe PORTB

    pop tmp                                                         // Se restauran registros usados
    pop segpat
    pop aux2
    pop aux
    ret                                                             // Fin de rutina

/**************/                                                    
Delay20:                                                            // Delay por software (~20ms aprox) para anti-rebote
    push r20                                                        // Se preservan r20..r22 porque se usan como contadores internos
    push r21
    push r22

    ldi r20, 2                                                      // Loop externo
D_O:
    ldi r21, 50                                                     // Loop medio
D_M:
    ldi r22, 255                                                    // Loop interno
D_I:
    dec r22                                                         // Cuenta hacia abajo
    brne D_I                                                        // Repite hasta 0
    dec r21
    brne D_M
    dec r20
    brne D_O

    pop r22                                                         // Se restauran registros originales
    pop r21
    pop r20
    ret                                                             // Fin del delay
