import serial
import serial.tools.list_ports
import struct
from datetime import datetime
from pathlib import Path

# ─── Размер FpgaToEspPacket_t (pragma pack(1)) ────────────────────────────────
# uint32 start_marker       = 4
# uint32 packet_counter     = 4
# uint16 status (GroupStatus_t union)  = 2
# uint16 alarms (GroupAlarms_t union)  = 2
# GroupInput_t:   10 x uint16         = 20
# GroupOutput_t:  17 x uint16         = 34
# GroupBattery_t:  6 x uint16         = 12
# uint32 crc32              = 4
# uint32 system_time_ms     = 4
# ИТОГО:                              = 86 байт
# ModulData_t = union { FpgaToEspPacket_t; uint8_t Tx_Buffer[86] } = 86 байт

FRAME_FORMAT = "<II HH 10H 17H 6H II"
FRAME_SIZE   = struct.calcsize(FRAME_FORMAT)

# ─── Флаги статуса (GroupStatus_t, биты 0–10) ─────────────────────────────────
STATUS_FLAGS = {
    0:  ("WARN",  "[10001] Авария электросети на входе"),
    1:  ("WARN",  "[10002] Авария электросети байпаса"),
    2:  ("INFO",  "[10003] Выпрямитель: работает"),
    3:  ("INFO",  "[10004] Инвертор: работает"),
    4:  ("INFO",  "[10005] Питание через инвертор"),
    5:  ("INFO",  "[10006] Питание по байпасу"),
    6:  ("WARN",  "[10007] Рассогласование синхронизации"),
    7:  ("INFO",  "[10008] Нагрузка от инвертора"),
    8:  ("WARN",  "[10009] Звуковая сигнализация"),
    9:  ("INFO",  "[10010] АКБ: разряд"),
    10: ("INFO",  "[10011] ИБП: работа от батареи"),
}

# ─── Флаги аварий (GroupAlarms_t, биты 0–10) ──────────────────────────────────
ALARM_FLAGS = {
    0:  ("ERR",  "[10012] Низкое напряжение на входе ИБП"),
1:  ("ERR",  "[10013] Высокое напряжение DC шины"),
    2:  ("ERR",  "[10014] Низкий заряд АКБ"),
    3:  ("ERR",  "[10015] АКБ не подключены"),
    4:  ("ERR",  "[10016] Неисправность инвертора"),
    5:  ("ERR",  "[10017] Перегрузка инвертора по току"),
    6:  ("ERR",  "[10018] Высокое напряжение на выходе"),
    7:  ("ERR",  "[10019] Неисправность вентилятора"),
    8:  ("WARN", "[10020] Необходимо заменить АКБ"),
    9:  ("ERR",  "[10021] Перегрев выпрямителя"),
    10: ("ERR",  "[10022] Перегрев инвертора"),
}

# ─── ANSI цвета ───────────────────────────────────────────────────────────────
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
GREEN  = "\033[92m"
GRAY   = "\033[90m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def parse_frame(raw: bytes) -> dict | None:
    if len(raw) < FRAME_SIZE:
        return None
    f = struct.unpack_from(FRAME_FORMAT, raw, 0)
    i = 0
    pkt = {}
    pkt["start_marker"]   = f[i]; i += 1
    pkt["pkt_counter"]    = f[i]; i += 1
    pkt["status_raw"]     = f[i]; i += 1
    pkt["alarms_raw"]     = f[i]; i += 1

    inp = ["v_in_AB","v_in_BC","v_in_CA","v_bypass_A","v_bypass_B",
           "v_bypass_C","i_in_A","i_in_B","i_in_C","freq_in"]
    pkt["input"] = {n: f[i+j] for j,n in enumerate(inp)}; i += 10

    out = ["v_out_A","v_out_B","v_out_C","freq_out",
           "i_out_A","i_out_B","i_out_C",
           "p_active_A","p_active_B","p_active_C",
           "p_apparent_A","p_apparent_B","p_apparent_C",
           "load_pct_A","load_pct_B","load_pct_C","event_count"]
    pkt["output"] = {n: f[i+j] for j,n in enumerate(out)}; i += 17

    bat = ["bat_voltage","bat_capacity","bat_groups_count",
           "dc_bus_voltage","bat_current","backup_time"]
    pkt["battery"] = {n: f[i+j] for j,n in enumerate(bat)}; i += 6

    pkt["crc32"]          = f[i]; i += 1
    pkt["system_time_ms"] = f[i]; i += 1
    return pkt

def collect_active_flags(pkt: dict) -> list:
    flags = []
    for bit, (level, desc) in STATUS_FLAGS.items():
        if pkt["status_raw"] & (1 << bit):
            flags.append((level, desc))
    for bit, (level, desc) in ALARM_FLAGS.items():
        if pkt["alarms_raw"] & (1 << bit):
            flags.append((level, desc))
    return flags

def format_frame(frame_idx: int, pkt: dict, flags: list, ansi: bool = True) -> str:
    """Форматирует кадр в строку. ansi=False — без цветов (для файла)."""
    t_ms  = pkt["system_time_ms"]
    t_sec = t_ms / 1000.0
    inp   = pkt["input"]
    out   = pkt["output"]
    bat   = pkt["battery"]
    cur   = (pkt["battery"]["bat_current"] if pkt["battery"]["bat_current"] < 32768
             else pkt["battery"]["bat_current"] - 65536)

    B  = BOLD  if ansi else ""
    R  = RESET if ansi else ""
    rd = RED   if ansi else ""
    yw = YELLOW if ansi else ""
    cy = CYAN  if ansi else ""
    gr = GRAY  if ansi else ""

    lines = []
    lines.append(f"\n{'═'*70}")
    lines.append(f"  Кадр #{frame_idx:<5} │ Пакет #{pkt['pkt_counter']:<8} │ "
                 f"Время ESP32: {t_sec:>10.3f} с  ({t_ms} мс)")
    lines.append(f"{'═'*70}")

    lines.append(f"  ФЛАГИ:")
    for level, desc in flags:
        if level == "ERR":
            lines.append(f"    {rd}[АВАРИЯ]  {desc}{R}")
        elif level == "WARN":
            lines.append(f"    {yw}[ПРЕДУПР] {desc}{R}")
        else:
            lines.append(f"    {cy}[INFO]    {desc}{R}")

    lines.append(f"\n  ВХОД:")
    lines.append(f"    Напр. AB={inp['v_in_AB']/10:.1f}В  BC={inp['v_in_BC']/10:.1f}В  CA={inp['v_in_CA']/10:.1f}В")
    lines.append(f"    Байпас  A={inp['v_bypass_A']/10:.1f}В  B={inp['v_bypass_B']/10:.1f}В  C={inp['v_bypass_C']/10:.1f}В")
    lines.append(f"    Ток     A={inp['i_in_A']/10:.1f}А  B={inp['i_in_B']/10:.1f}А  C={inp['i_in_C']/10:.1f}А")
    lines.append(f"    Частота {inp['freq_in']/100:.2f} Гц")

    lines.append(f"\n  ВЫХОД:")
    lines.append(f"    Напр.   A={out['v_out_A']/10:.1f}В  B={out['v_out_B']/10:.1f}В  C={out['v_out_C']/10:.1f}В")
    lines.append(f"    Частота {out['freq_out']/100:.2f} Гц")
    lines.append(f"    Ток     A={out['i_out_A']/10:.1f}А  B={out['i_out_B']/10:.1f}А  C={out['i_out_C']/10:.1f}А")
    lines.append(f"    P акт.  A={out['p_active_A']/10:.1f}кВт  B={out['p_active_B']/10:.1f}кВт  C={out['p_active_C']/10:.1f}кВт")
    lines.append(f"    P полн. A={out['p_apparent_A']/10:.1f}кВА  B={out['p_apparent_B']/10:.1f}кВА  C={out['p_apparent_C']/10:.1f}кВА")
    lines.append(f"    Нагрузка A={out['load_pct_A']/10:.1f}%  B={out['load_pct_B']/10:.1f}%  C={out['load_pct_C']/10:.1f}%")
    lines.append(f"    Событий: {out['event_count']}")

    lines.append(f"\n  АКБ:")
    lines.append(f"    Напр.={bat['bat_voltage']/10:.1f}В  DC шина={bat['dc_bus_voltage']/10:.1f}В")
    lines.append(f"    Ёмкость={bat['bat_capacity']} А·ч  Групп={bat['bat_groups_count']}")
    lines.append(f"    Ток={cur/10:+.1f}А  Автономия={bat['backup_time']} мин")
    lines.append(f"\n  {gr}CRC32=0x{pkt['crc32']:08X}  start_marker=0x{pkt['start_marker']:08X}{R}")

    return "\n".join(lines)

def print_frame(frame_idx: int, pkt: dict, flags: list):
    print(format_frame(frame_idx, pkt, flags, ansi=True))

LOG_PATH = Path(__file__).parent / "dump_log.txt"

# Таймаут тишины (сек) — если данные перестали идти дольше этого времени,
# считаем что дамп завершён и сохраняем накопленное
DUMP_IDLE_TIMEOUT = 3.0

def save_dump_to_file(dump_frames: list, dump_number: int):
    """Очищает файл и записывает все накопленные кадры."""
    error_frames = sum(1 for _, flags in dump_frames if flags)

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write(f"{'='*70}\n")
        f.write(f"  ДАМП #{dump_number}  —  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  Кадров в дампе        : {len(dump_frames)}\n")
        f.write(f"  Кадров с флагами      : {error_frames}\n")
        f.write(f"{'='*70}\n")

        for frame_idx, (pkt, flags) in enumerate(dump_frames, start=1):
            f.write(format_frame(frame_idx, pkt, flags, ansi=False))
            f.write("\n")

    print(f"\n{GREEN}  Дамп #{dump_number} сохранён → {LOG_PATH}  "
          f"({len(dump_frames)} кадров, {error_frames} с флагами){RESET}")

def receive_dump(port: str, baudrate: int = 115200):
    print(f"\n{GREEN}Открываю {port} @ {baudrate}...{RESET}")
    print(f"{GREEN}Файл лога: {LOG_PATH}{RESET}")
    print(f"{GRAY}Жду данные... Ctrl+C для остановки{RESET}\n")

    with serial.Serial(port, baudrate, timeout=DUMP_IDLE_TIMEOUT) as ser:
        recv_buf     = bytearray()
        dump_frames  = []
        dump_number  = 0
        total_frames = 0

        try:
            while True:
                chunk = ser.read(FRAME_SIZE * 4)

                # Данные не пришли за DUMP_IDLE_TIMEOUT секунд
                if not chunk:
                    if dump_frames:
                        # Тишина после данных — дамп завершён, сохраняем
                        dump_number += 1
                        save_dump_to_file(dump_frames, dump_number)
                        dump_frames = []
                        print(f"{GRAY}  Жду следующий дамп...{RESET}\n")
                    else:
                        print(f"\r{GRAY}  Ожидание данных...{RESET}",
                              end="", flush=True)
                    continue

                recv_buf.extend(chunk)

                while len(recv_buf) >= FRAME_SIZE:
                    raw = bytes(recv_buf[:FRAME_SIZE])
                    pkt = parse_frame(raw)

                    if pkt is None:
                        recv_buf.pop(0)
                        continue

                    if pkt["start_marker"] != 0xAA55AA55:
                        recv_buf.pop(0)
                        continue

                    # Кадр валидный
                    recv_buf = recv_buf[FRAME_SIZE:]
                    total_frames += 1

                    flags = collect_active_flags(pkt)
                    dump_frames.append((pkt, flags))

                    # Прогресс в консоли
                    print(f"\r{GRAY}  Получено кадров: {len(dump_frames)}{RESET}",
                          end="", flush=True)

                    # Кадры с флагами — сразу показываем в консоли
                    if flags:
                        print()
                        print_frame(len(dump_frames), pkt, flags)

        except KeyboardInterrupt:
            # Если при выходе были накоплены кадры — сохраняем их
            if dump_frames:
                dump_number += 1
                save_dump_to_file(dump_frames, dump_number)

            print(f"\n\n{BOLD}{'═'*70}{RESET}")
            print(f"{BOLD}  Остановлено.{RESET}")
            print(f"  Всего кадров получено  : {total_frames}")
            print(f"  Сохранённых дампов     : {dump_number}")
            print(f"{BOLD}{'═'*70}{RESET}\n")

def select_port() -> str:
    ports = serial.tools.list_ports.comports()
    if not ports:
        print(f"{RED}COM-порты не найдены!{RESET}")
        exit(1)
    print("\nДоступные COM-порты:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device:10} — {p.description}")
    if len(ports) == 1:
        print(f"\nАвтовыбор: {ports[0].device}")
        return ports[0].device
    choice = input("\nВыберите номер порта: ").strip()
    return ports[int(choice)].device

if __name__ == "__main__":
    print(f"Размер кадра FpgaToEspPacket_t: {FRAME_SIZE} байт")
    port = select_port()
    receive_dump(port, baudrate=115200)