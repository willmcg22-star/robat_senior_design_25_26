from batbot_bringup.bb_tendons.TendonHardware import TendonHardwareInterface, OPCODE
import sys
import timeit

def timing(port_name, num_trials=200):
    th = TendonHardwareInterface(port_name=port_name)

    total_times_sum = 0
    packet_build_time_sum = 0
    transmit_time_sum = 0
    read_parse_time_sum = 0

    for i in range(0, num_trials):

        packet_build_time = 0
        transmit_time = 0
        read_parse_time = 0

        start = timeit.default_timer()
        params = [0, 0]
        th.BuildPacket(0, OPCODE.WRITE_ANGLE.value, params)
        end = timeit.default_timer()
        packet_build_time = end - start

        start = timeit.default_timer()
        th.SendTx()
        end = timeit.default_timer()
        transmit_time = end - start

        start = timeit.default_timer()
        th.ReadRx()
        end = timeit.default_timer()
        read_parse_time = end - start

        total_times_sum += (packet_build_time + transmit_time + read_parse_time)
        packet_build_time_sum += packet_build_time
        transmit_time_sum += transmit_time
        read_parse_time_sum += read_parse_time

    avg_total_times_sum = total_times_sum / num_trials
    avg_packet_build_time_sum = packet_build_time_sum / num_trials
    avg_transmit_time_sum = transmit_time_sum / num_trials
    avg_read_parse_time_sum = read_parse_time_sum / num_trials

    print(f"Average Transaction Time Breakdown over {num_trials} Transactions")
    print(f"Average Transaction Time (ms): {(avg_total_times_sum * 1000):.3f}")
    print(f"Average Packet Build Time (ms): {(avg_packet_build_time_sum * 1000):.3f}")
    print(f"Average Packet Transmit Time (ms): {(avg_transmit_time_sum * 1000):.3f}")
    print(f"Average Packet Read Time (ms): {(avg_read_parse_time_sum * 1000):.3f}")

if __name__ == "__main__":

    try:
        if len(sys.argv) < 2:
            raise ValueError("No port name provided, please provide it!\n\nExample: python tendon_time_profiling.py [port_name]\n")

        port_name = sys.argv[1]

        if len(sys.argv) == 3:
            num_trials = int(sys.argv[2])
            timing(port_name=port_name, num_trials=num_trials)
        else:
            timing(port_name=port_name)
    except Exception as e:
        print(e)