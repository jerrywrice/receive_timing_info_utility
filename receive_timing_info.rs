//! This example performs a receive timing test using real hardware ports.
//!
//! This tool serves measures serial port receive processing timing under various scenarios and parameters:
//!      various baud-rates, write and read invocation orders, write lengths, read request lengths, etc...
//!
//! This example opens two comm ports (a xmt and rcv port) which are connected with a 
//!  cross-over (null modem).
//!  The command line arguments are => 
//!  
//!     1. The transmit port name          => --txport="port-name"
//!     2. The receive  port name          => --rxport="port-name"
//!     3. The timing results logfile name => --log="file-path"
//!     4. The baud rate                   => --baud="bbbb" where bbbb is a positive baud rate integer
//!     5. The rxport read time-out in ms  => --rxtmo="tttt" where tttt is an unsigned integer timeout setting (in ms).
//!     6. The ms delay after a write()    => --posttxdelayms='tttt' where tttt is an unsigned integer delay value (in ms). Defaults to 0.
//!     7. The read() and write() 'stalled' timeout in ms    
//!                                        => --xfrstalledtmo='tttt' where tttt is an unsigned integer timeout value (in ms). Defaults to the higher of 1000 ms or (4 * rxtmo).
//!     8. The tx data length              => --txlen="nnnn" where is a positive integer.
//!     9. The rx data length              => --rxlen="nnnn" where is a positive integer. 
//!     10. An optional repeat count       => --repeat="nnnn" where nnnn is a positive repeat count. Defaults to 1.
//!     11. Enable extra debug logging     => --fulldbg="Y[es]" or "N[o]" when Yes logs markers around invocations of read() to 
//!                                                                          support detection of an inifinitely blocked read().
//!                                                                          The need for this specific debugging log feature will
//!                                                                          be eliminated once a more sophisticated multi-threaded
//!                                                                          version of this test application is implemented. Defaults to "No". 

//!  Example instance invocations are 

//!     `receive_timing_info --txport=COM5 --rxport=COM6 --baud=115200 --log="D:/filename.log" --rxtmo=20 --aftertxdelayms=0 --txlen=10 --rxlen=20 --repeat=10 --fulldbg=YES'
//!    
use core::time;
use std::env;
use fast_log;
use fast_log::Config;

#[cfg(windows)]
use winapi::um::winuser::MessageBeep;

use std::{io::Write};
use std::{str, thread, };
//use std::sync::Mutex;
use std::time::Duration;
use std::time::Instant;

use clap::{Arg, Command};

use serialport::{ClearBuffer /* , DataBits, FlowControl, Parity, SerialPort, StopBits */ };

static SEQUENCED_U8S: [u8; 256] = [000u8,001u8,002u8,003u8,004u8,005u8,006u8,007u8,008u8,009u8,010u8,011u8,012u8,013u8,014u8,015u8,016u8,017u8,018u8,019u8,020u8,021u8,022u8,023u8,024u8,025u8,026u8,027u8,028u8,029u8,030u8,031u8,
032u8,033u8,034u8,035u8,036u8,037u8,038u8,039u8,040u8,041u8,042u8,043u8,044u8,045u8,046u8,047u8,048u8,049u8,050u8,051u8,052u8,053u8,054u8,055u8,056u8,057u8,058u8,059u8,060u8,061u8,062u8,063u8,
064u8,065u8,066u8,067u8,068u8,069u8,070u8,071u8,072u8,073u8,074u8,075u8,076u8,077u8,078u8,079u8,080u8,081u8,082u8,083u8,084u8,085u8,086u8,087u8,088u8,089u8,090u8,091u8,092u8,093u8,094u8,095u8,
096u8,097u8,098u8,099u8,100u8,101u8,102u8,103u8,104u8,105u8,106u8,107u8,108u8,109u8,110u8,111u8,112u8,113u8,114u8,115u8,116u8,117u8,118u8,119u8,120u8,121u8,122u8,123u8,124u8,125u8,126u8,127u8,
128u8,129u8,130u8,131u8,132u8,133u8,134u8,135u8,136u8,137u8,138u8,139u8,140u8,141u8,142u8,143u8,144u8,145u8,146u8,147u8,148u8,149u8,150u8,151u8,152u8,153u8,154u8,155u8,156u8,157u8,158u8,159u8,
160u8,161u8,162u8,163u8,164u8,165u8,166u8,167u8,168u8,169u8,170u8,171u8,172u8,173u8,174u8,175u8,176u8,177u8,178u8,179u8,180u8,181u8,182u8,183u8,184u8,185u8,186u8,187u8,188u8,189u8,190u8,191u8,
192u8,193u8,194u8,195u8,196u8,197u8,198u8,199u8,200u8,201u8,202u8,203u8,204u8,205u8,206u8,207u8,208u8,209u8,210u8,211u8,212u8,213u8,214u8,215u8,216u8,217u8,218u8,219u8,220u8,221u8,222u8,223u8,
224u8,225u8,226u8,227u8,228u8,229u8,230u8,231u8,232u8,233u8,234u8,235u8,236u8,237u8,238u8,239u8,240u8,241u8,242u8,243u8,244u8,245u8,246u8,247u8,248u8,249u8,250u8,251u8,252u8,253u8,254u8,255u8
];

static PROGRESS_TEXT : &str = "+";

fn test_progress_warning(rxlen : usize, txlen : usize, rxtmo : usize) { // Currently only applies to Windows
    if rxlen > txlen {
        if rxtmo == 0 {
            println!(" $$$ DEADLOCK: rxlen({}) > txlen({}) when rxtmo == {} seconds. Windows serialport-rs read() blocks indefinitely.!!!!\n", rxlen, txlen, rxtmo / 1000);
        } else if rxtmo > 5000usize {
            println!(" !!!! Beware: rxlen({}) > txlen({}) while rxtmo == {} seconds. Test progress may be painfully slow!!!!\n", rxlen, txlen, rxtmo / 1000);
        }
    }
}

fn main() {
    let matches = Command::new("Serialport Example Test - Bulk Xfr Check")
        .about("Measures serial port read and write timings.")
        .disable_version_flag(true)
        .arg(Arg::new("txport")
            .help("The device path of the serial port that sends serial data.")
            .require_equals(true)
            .takes_value(true)
            .long("txport")
            .required(true))
        .arg(Arg::new("rxport")
            .help("The device path of the serial port that receives serial data.")
            .require_equals(true)
            .takes_value(true)
            .long("rxport")
            .required(true))
        .arg(Arg::new("baud")
            .help("The serial baud rate.")
            .require_equals(true)
            .takes_value(true)
            .long("baud")
            .required(true))
       .arg(Arg::new("log")
             .help("The test results log filename.")
             .require_equals(true)
             .takes_value(true)
             .long("log")
             .required(true))
        .arg(Arg::new("rxtmo")
             .help("The rxport read timeout setting in ms.")
             .require_equals(true)
             .takes_value(true)
             .long("rxtmo")
             .required(true))
        .arg(Arg::new("posttxdelayms")
             .help("Delay after write in ms.")
             .require_equals(true)
             .takes_value(true)
             .long("posttxdelayms")
             .required(false))
        .arg(Arg::new("xfrstalledtmo")
             .help("read() or write() stalled timeout period in ms.")
             .require_equals(true)
             .takes_value(true)
             .long("xfrstalledtmo")
             .required(false))
        .arg(Arg::new("txlen")
             .help("The number of bytes transferred.")
             .require_equals(true)
             .takes_value(true)
             .long("txlen")
             .required(true))
        .arg(Arg::new("rxlen")
             .help("The read() request byte count.")
             .require_equals(true)
             .takes_value(true)
             .long("rxlen")
             .required(true))
        .arg(Arg::new("repeat")
             .help("The total number  of repeated bulk transfer transfers. Defaults to 1.")
             .require_equals(true)
             .takes_value(true)
             .long("repeat")
             .required(false))
        .arg(Arg::new("fulldbg")
             .help("Log entry and return marker text lines are written when invoking read(). Helps interpret infinite pending. 'Y[es]' or 'N[o]'. Defaults to 'No'.")
             .require_equals(true)
             .takes_value(true)
             .long("fulldbg")
             .required(false))
        .get_matches();          

    // Command line argument values
    let txport_name = matches.value_of("txport").unwrap();
    let rxport_name = matches.value_of("rxport").unwrap();
    let log_name = matches.value_of("log").unwrap();
    let baud_val_str = matches.value_of("baud").unwrap();
    let baud_val: u32 = baud_val_str.replace("_","").parse().unwrap();	
    let rxtmo_val_str = matches.value_of("rxtmo").unwrap();
    let rxtmo_val: u32 = rxtmo_val_str.replace("_","").parse().unwrap();
    let posttxdelayms_str : &str = matches.value_of("posttxdelayms").unwrap_or("0");
    let posttxdelayms : u32 = posttxdelayms_str.replace("_","").parse().unwrap();
    let xfrstalledtmo_ms_str : &str = matches.value_of("xfrstalledtmo").unwrap_or("1000");
    let mut xfrstalledtmo_ms : u32 = xfrstalledtmo_ms_str.replace("_","").parse().unwrap();
    let txlen_val_str = matches.value_of("txlen").unwrap();
    let txlen_val: usize = txlen_val_str.replace("_","").parse().unwrap();	
    let rxlen_val_str = matches.value_of("rxlen").unwrap();
    let rxlen_val: usize = rxlen_val_str.replace("_","").parse().unwrap();	
    let xfr_repeat_val_str = matches.value_of("repeat").unwrap_or("1");
    let xfr_repeat_val: u32 = xfr_repeat_val_str.parse().unwrap();	
    let fulldbg_str : &str= matches.value_of("fulldbg").unwrap_or(&"No");
    let b_full_debug_logging : bool = fulldbg_str.starts_with(&"Y") || fulldbg_str.starts_with(&"y"); 

    fast_log::init(Config::new().file(log_name).chan_len(Some(100000))).unwrap();

    // Clamp xfrstalledtmo_ms to the higher of 1000 or 4 * rxtmo_val (ms)
    if xfrstalledtmo_ms < 1000 {
        xfrstalledtmo_ms = 1000;
    }
    if xfrstalledtmo_ms < (rxtmo_val * 4) {
        xfrstalledtmo_ms = rxtmo_val * 4;
    }

    log::info!("'receive_timing_info' cross platform dual RS-232 port null modem cable connected rcv+xmt+timeout test and characterization tool: v1.0");
    println!("'receive_timing_info' cross platform dual RS-232 port null modem cable connected rcv+xmt+timeout test and characterization tool: v1.0");

    log::info!("Test setup: Platform='{}', Baud={}, rxtmo={} ms, posttxdelayms={} ms, xfrstalledtmo={} ms, txlen={}, rxlen={}, repeat={}, fulldbg={}", env::consts::OS, baud_val, rxtmo_val, posttxdelayms, xfrstalledtmo_ms, txlen_val, rxlen_val, xfr_repeat_val, b_full_debug_logging );
    println!("Test setup: Platform='{}', Baud={}, rxtmo={} ms, posttxdelayms={} ms, xfrstalledtmo={} ms, txlen={}, rxlen={}, repeat={}, fulldbg={}", env::consts::OS, baud_val, rxtmo_val, posttxdelayms, xfrstalledtmo_ms, txlen_val, rxlen_val, xfr_repeat_val, b_full_debug_logging );
    log::info!("Test Logfile Name: '{}'", log_name);
    println!("Test Logfile Name: '{}'", log_name );
    
    //println!("\nThe test measurement data is written exclusively to log-file rather than screen to avoid undue latency which skews timing data.\n");
    //println!("     ** Warning: During testing should 'receive_timing_info' appear to hang, then either the read() or write() ");
    //println!("     **          have blocked indefinitely. This can result from supplying a very large 'rxtmo' parameter combined ");
    //println!("     **          with 'txlen' < 'rxlen' parameters. Otherwise it can be a problem with your hardware (cable unplugged),");
    //println!("     **          or a 'bug' in the target serialport-rs logic. If this occurs with reasonable parameters and the cables are");
    //println!("     **          connected properly, consider re-running with 'fulldbg=Y' enabled and examine the resulting log-file.");
    //println!("     **          Optional parameter 'fulldbg=Y' inserts log markers around 'write()' and 'read()' invocations, making it");
    //println!("     **          obvious from the log at which point the test code has blocked. Obviously, this also extends certain timing metrics.\n");

    #[cfg(windows)]
    test_progress_warning(rxlen_val, txlen_val, rxtmo_val as usize);
   
    // Display initial progress marker
    if b_full_debug_logging {
        invoke_progress_indicator();
    }

    let mut repeat_loop_cnt = 0;
    let mut txbuf : Vec<u8> = Vec::with_capacity(txlen_val);
    for index in 0..txlen_val { // Initialize and extend 'txbuf' with ascending sequenced bytes cycling in value from 0 to 0xff for 'txlen' bytes
        txbuf.push(SEQUENCED_U8S[index & 255]);
    }
    let mut rxbuf : Vec<u8> = Vec::with_capacity(rxlen_val);
    for _index in 0..rxlen_val { // Define and extend the rx buffer to 'rxlen' byte
        rxbuf.push(0);
    }

    let mut cycle_initial_phase = true;

    while repeat_loop_cnt < xfr_repeat_val{
        // Open the appropriate (alternate) tx and rx ports
        let tx_port_name = if cycle_initial_phase { txport_name.clone() } else { rxport_name.clone() }; // if resume_same_cycle then switch tx and rx ports
        let rx_port_name = if cycle_initial_phase { rxport_name.clone() } else { txport_name.clone() }; // if resume_same_cycle then switch tx and rx ports

        // Open tx port 
        let mut txport = match serialport::new(tx_port_name, baud_val as u32).open() {
            Err(e) => {
                eprintln!("Failed opening tx port \"{}\". Error: {}", tx_port_name, e);
                ::std::process::exit(1);
            }
            Ok(p) => p,
        };

        // Open rx port 
        let mut rxport = match serialport::new(rx_port_name, baud_val as u32).open() {
            Err(e) => {
                eprintln!("Failed opening rx port \"{}\". Error: {}", rx_port_name, e);
                ::std::process::exit(1);
            }
            Ok(p) => p,
        };

        // Log the active rx and tx port names =>
        if cycle_initial_phase {
            log::info!("");
            log::info!("** Start of cycle {}. **",repeat_loop_cnt+1);
            log::info!("Cycle {} first phase -> Rx port = '{}', Tx port = '{}' .",repeat_loop_cnt+1,rx_port_name,tx_port_name);
        } else /* cycle final phase */ {
            log::info!("Cycle {} second phase uses reversed rx and tx directions -> Rx port = '{}', Tx port = '{}' .",repeat_loop_cnt+1,rx_port_name,tx_port_name);
        }

        //   Set the RX port receive timeout
        rxport.set_timeout(Duration::new(rxtmo_val as u64 / 1000, (rxtmo_val % 1000) * 1_000_000  /* ms  */)).unwrap();
        txport.set_timeout(Duration::new(rxtmo_val as u64 / 1000, (rxtmo_val % 1000) * 1_000_000  /* ms  */)).unwrap();

        // Transmit and measure elapsed time of read() invocation, and read return len.
   
        // Assure no residual unread transmit data is in the pipe from prior (repeat) cycle
        txport.clear(ClearBuffer::All).unwrap();
        rxport.clear(ClearBuffer::All).unwrap();
        thread::sleep(time::Duration::from_millis(200));
        txport.clear(ClearBuffer::All).unwrap();
        rxport.clear(ClearBuffer::All).unwrap();

        for index in 0..rxlen_val { // Preset the rx buffer with known pattern different that what is to be transmitted
            rxbuf[index] = 0xffu8;
        }
     
        let mut write_operation_timer_initial_instant : Instant = Instant::now();

        let mut actual_total_tx_data_written = 0;
        let mut actual_total_rx_data_received = 0;
        let mut current_serial_log_entry_line = "".to_string();
        let mut total_expected_rcv_count_limit = rxlen_val;
        if txlen_val < rxlen_val {
            total_expected_rcv_count_limit = rxlen_val; // txlen_val;
        }

        // Setup initial stalled time-out timers for write() and read().
        let mut write_stalled_timer_active = true;
        let mut read_stalled_timer_active = true;
        let mut initial_stalled_write_timer_instant : Instant = Instant::now();
        let mut initial_stalled_read_timer_instant = Instant::now();
        
        while  (actual_total_tx_data_written < txlen_val) || (actual_total_rx_data_received < total_expected_rcv_count_limit) { 
            // Transmit data section ...
            if write_stalled_timer_active {
                let current_stalled_write_timer_elapsed_duration = Instant::now().duration_since(initial_stalled_write_timer_instant);
                if current_stalled_write_timer_elapsed_duration.as_millis() >= xfrstalledtmo_ms as u128 {
                    // write() stall timeout has been detected. Report and exit test.
                    log::info!("\nTRANSFER STALLED TIMEOUT ERROR: 'txport::write()' repeatedly timed-out while trying to transmit its out-going data. If not induced, inspect-verify the serial connections. Aborting.");
                    eprintln!("\nTRANSFER STALLED TIMEOUT ERROR: 'txport::write()' repeatedly timed-out while trying to transmit its out-going data. If not induced, inspect-verify the serial connections. Aborting.");
                    spin_sleep::sleep(Duration::from_secs(2));
                    std::process::exit(-1);
                }
            }

            if read_stalled_timer_active {
                let current_stalled_read_timer_elapsed_duration = Instant::now().duration_since(initial_stalled_read_timer_instant);
                if current_stalled_read_timer_elapsed_duration.as_millis() >= xfrstalledtmo_ms as u128 {
                    // read() stall timeout has been detected. Report and exit test.
                    log::info!("\nTRANSFER STALLED TIMEOUT ERROR: 'rxport::read()' repeatedly timed-out without receiving its requested incoming data. If not induced, inspect+verify the serial connections. Aborting.");
                    eprintln!("\nTRANSFER STALLED TIMEOUT ERROR: 'rxport::read()' repeatedly timed-out without receiving its requested incoming data. If not induced, inspect+verify the serial connections. Aborting.");
                    spin_sleep::sleep(Duration::from_secs(2));
                    std::process::exit(-1);
                }
            }

            if actual_total_tx_data_written < txlen_val {
                let mut transmitted : bool = false;
                while !transmitted {
                    let this_tx_len;
                    let actual_write_duration : Duration;
                    // So transmit more data

                    if b_full_debug_logging {
                        let write_marker_with_bufferrange : String = "Enter write(txbuf[".to_string() + &actual_total_tx_data_written.to_string() + ".." + &txlen_val.to_string() + "]).";
                        log::info!("{}",write_marker_with_bufferrange);
                    }
        
                    let start_write_instant : Instant = Instant::now();
                    let txresult = txport.write(&txbuf[actual_total_tx_data_written..txlen_val]);
                    txport.flush().unwrap();
                    actual_write_duration = Instant::now().duration_since(start_write_instant);
                    write_operation_timer_initial_instant = Instant::now();

                    if b_full_debug_logging {
                        log::info!("Return from write().");
                    }
        
                    match txresult {
                        Ok(len) => {
                            actual_total_tx_data_written += len;
                            this_tx_len = len;
                            transmitted = true;
                            write_stalled_timer_active = true; // Make sure the timeout timer is flagged as active
                            initial_stalled_write_timer_instant = Instant::now(); // Refresh write() stall timer for this successful write() attempt.
                        },
                        Err(e) => {
                            eprintln!("Error transmitting => \'{e}\'");
                            std::process::exit(-1);
                        }
                    }

                    current_serial_log_entry_line.push_str("txport.write() sent ");
                    current_serial_log_entry_line += &this_tx_len.to_string();
                    current_serial_log_entry_line.push_str(" bytes while blocked for ");
                    current_serial_log_entry_line += &actual_write_duration.as_micros().to_string();
                    current_serial_log_entry_line.push_str(" us. Read() invoked "); 
                    if posttxdelayms > 0 {
                        let rx_delay_milli = time::Duration::from_millis(posttxdelayms as u64);
                        spin_sleep::sleep(rx_delay_milli);
                    }
                    read_stalled_timer_active = true; // Reactivate read() stall timer for the corresponding read() attempt.
                    initial_stalled_read_timer_instant = Instant::now(); // Reactivate read() stall timer for the corresponding read() attempt.
                }
            }
            else {
                write_stalled_timer_active = false; // Disable the write() stalled timeout timer since all data for this half-cycle has been sent.
            }

            // Should we perform another read() here - only if outstanding transmitted data
            if actual_total_tx_data_written < actual_total_rx_data_received {
                read_stalled_timer_active = false; // Disable the read() stalled timeout timer since there is no outstanding unread data which has been sent.
                continue; // No outstanding receive data to attempt to read now, so bypass receive this loop iteration
            }

            // Receive data section ...
            let mut this_rx_request_len:usize = rxlen_val - actual_total_rx_data_received;
            let mut this_rx_len = 0usize;
            
            // Account for the caller's rxlen parameter value being less than the txlen parameter value.
            let mut rcv_status_info = "".to_string();

            // ... receive more data

            // Determine if this is a subsequent receive with no immediately preceeding write ()
            //   If so, then offset the log text to line up properly in the logfile (for better readibility)
            if current_serial_log_entry_line.len() == 0 {
                current_serial_log_entry_line = "                                                      Read() invoked ".to_string(); 
            }

            if b_full_debug_logging {
                let read_marker_with_bufferrange : String = "Enter read(rxbuf[".to_string() + &actual_total_rx_data_received.to_string() + ".." + &rxlen_val.to_string() + "]).";
                log::info!("{}",read_marker_with_bufferrange);
            }
            
            // Perform the read() operation

            let actual_read_duration;
            let elapsed_us_since_write = Instant::now().duration_since(write_operation_timer_initial_instant);
            let start_read_instant : Instant = Instant::now();
            let rxresult = rxport.read(&mut rxbuf[actual_total_rx_data_received..rxlen_val]);
            actual_read_duration = Instant::now().duration_since(start_read_instant);
            if b_full_debug_logging {
                log::info!("Return from read().");
            }
            match rxresult {
                Ok(len) => {
                    this_rx_len = len;
                    actual_total_rx_data_received += this_rx_len;
                    read_stalled_timer_active = true; // Make sure the read timeout timer is active after this successful read() attempt.
                    initial_stalled_read_timer_instant = Instant::now(); // Refresh read() stall timer for this read() successful attempt.
                },
                Err(e) => {
                    if e.kind() != std::io::ErrorKind::TimedOut {
                        eprintln!("Error receiving => \'{e}\'");
                        std::process::exit(-1);
                    }
                    else {
                        rcv_status_info = "Rcv timeout.".to_string();
                    }
                }
            }

            // Log read() invocation measurements
            current_serial_log_entry_line += &elapsed_us_since_write.as_micros().to_string();
            current_serial_log_entry_line.push_str(" us after write(), rxport.read(");
            let read_rqst_len : usize = 0;
            current_serial_log_entry_line += &this_rx_request_len.to_string();
            current_serial_log_entry_line.push_str(") returned ");
            current_serial_log_entry_line += &this_rx_len.to_string();
            current_serial_log_entry_line.push_str(" bytes while blocked for ");
            current_serial_log_entry_line += &actual_read_duration.as_micros().to_string();
            current_serial_log_entry_line.push_str(" us. ");
            if rcv_status_info.len() > 0 {
                current_serial_log_entry_line += &rcv_status_info;
            }
            
            if b_full_debug_logging {
                invoke_progress_indicator();
            }

            log::info!("{}", current_serial_log_entry_line);
            current_serial_log_entry_line = "".to_string(); // Clear next log string buffer
        }
        // Verify total received data buffer matches transmitted data pattern
        for index in 0..total_expected_rcv_count_limit {
            if rxbuf[index] != txbuf[index] {
                current_serial_log_entry_line = "ERROR: rxbuf[".to_string() + &index.to_string() + &"] == '".to_string();
                current_serial_log_entry_line += rxbuf[index].to_string().as_str();
                current_serial_log_entry_line += "', txbuf[".to_string().as_str();
                current_serial_log_entry_line += index.to_string().as_str();
                current_serial_log_entry_line += "] == ".to_string().as_str();
                current_serial_log_entry_line += txbuf[index].to_string().as_str();
                current_serial_log_entry_line += "] - mismatch!";
                log::info!("{}", current_serial_log_entry_line);
                eprintln!("{current_serial_log_entry_line}");
                spin_sleep::sleep(Duration::from_secs(2));
                std::process::exit(-1);
            }
        }
        drop(rxport);
        drop(txport);

        if !cycle_initial_phase {
            log::info!("** End of cycle {}. **",repeat_loop_cnt+1);
            repeat_loop_cnt += 1;
        }
        cycle_initial_phase = !cycle_initial_phase; // Toggle    
    }
    log::info!(""); // Add blank log line
    log::info!("All test cycles (1..{}) passed! The received data patterns matched precisely those transmitted.",xfr_repeat_val);
    eprintln!("\n\nAll test cycles (1..{}) passed! The received data patterns matched precisely those transmitted.",xfr_repeat_val);
}

fn invoke_progress_indicator() {
    //#[cfg(windows)]
    //unsafe { MessageBeep(0xFFFFFFFF); } 
    //#[cfg(windows)]
    //spin_sleep::sleep(Duration::from_secs(1));
    eprint!("{}",PROGRESS_TEXT); // screen progress indicator
}


