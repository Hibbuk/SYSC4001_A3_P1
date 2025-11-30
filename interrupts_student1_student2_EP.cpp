/**
 * @file interrupts_student1_student2_EP.cpp
 * @author Your Names Here
 * @brief External Priorities Scheduler (No Preemption)
 */

#include "interrupts_student1_student2.hpp"

void FCFS(std::vector<PCB> &ready_queue) {
    std::sort( 
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time > second.arrival_time); 
                } 
            );
}

//sorts by PID (lower PID = higher priority, goes to back of vector)
void priority_sort(std::vector<PCB> &queue) {
    std::sort( 
                queue.begin(), queue.end(), 
                []( const PCB &first, const PCB &second ){
                    return (first.PID > second.PID);
                }
            );
}

std::tuple<std::string /* add std::string for bonus mark */ > run_simulation(std::vector<PCB> list_processes) {

    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes
    std::vector<PCB> job_list;      //A list to keep track of all the processes. This is similar
                                    //to the "Process, Arrival time, Burst time" table that you
                                    //see in questions. You don't need to use it, I put it here
                                    //to make the code easier :).

    unsigned int current_time = 0;
    PCB running;

    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    //Loop while till there are no ready or waiting processes.
    //This is the main reason I have job_list, you don't have to use it.
    while(job_list.empty() || !all_process_terminated(job_list)){

        //Inside this loop, there are three things you must do:
        // 1) Populate the ready queue with processes as they arrive
        // 2) Manage the wait queue
        // 3) Schedule processes from the ready queue

        //Population of ready queue is given to you as an example.
        //Go through the list of proceeses
        for(auto &process : list_processes) {
            if(process.arrival_time <= current_time && process.state == NOT_ASSIGNED) {
                //update the state to a new state because it has entered the queue
                process.state = NEW;
                
                if(assign_memory(process)) {
                    process.state = READY;  //Set the process state to READY
                    ready_queue.push_back(process); //Add the process to the ready queue
                    execution_status += print_exec_status(current_time, process.PID, NEW, READY);
                }
                job_list.push_back(process); //Add it to the list of processes
                
            } else if(process.arrival_time <= current_time && process.state == NEW) {
                //try to assign NEW state memory
                if(assign_memory(process)) {
                    process.state = READY;  //Set the process state to READY
                    ready_queue.push_back(process); //Add the process to the ready queue
                    sync_queue(job_list, process); // updates the process in job list
                    execution_status += print_exec_status(current_time, process.PID, NEW, READY);
                }
            }
        }

        ///////////////////////MANAGE WAIT QUEUE/////////////////////////
        //This mainly involves keeping track of how long a process must remain in the wait queue
        for (auto iter = wait_queue.begin(); iter != wait_queue.end(); ) {
            iter->io_remaining--;
            if(iter->io_remaining < 1) {
                //change the state that has no more time to wait to READY
                iter->state = READY;
                ready_queue.push_back(*iter);
                sync_queue(job_list, *iter);
                execution_status += print_exec_status(current_time, iter->PID, WAITING, READY);

                //Erase the state from the wait queue after you do everything you need to with it.
                iter = wait_queue.erase(iter);
            } else {
                ++iter;
            }
        }
        /////////////////////////////////////////////////////////////////

        
        //Manage running process
        if(running.state == RUNNING) {
            
            // Decrement first
            running.remaining_time--;
            
            // Then check if process completed
            if(running.remaining_time == 0) {
                running.completion_time = current_time;
                terminate_process(running, job_list);
                sync_queue(job_list, running);
                execution_status += print_exec_status(current_time, running.PID, RUNNING, TERMINATED);
                idle_CPU(running);
            }
            // Check if the time passed % I/O Frequency, then you know that an I/O occurs
            else if(running.io_freq > 0 && (running.processing_time - running.remaining_time) > 0 && 
                    ((running.processing_time - running.remaining_time) % running.io_freq) == 0) {
                
                //change current program to WAITING and add it to the wait_queue    
                running.state = WAITING;
                running.io_remaining = running.io_duration;
                wait_queue.push_back(running);
                sync_queue(job_list, running);
                execution_status += print_exec_status(current_time, running.PID, RUNNING, WAITING);
                idle_CPU(running);
            }
        }

        //////////////////////////SCHEDULER//////////////////////////////
        if(running.state == NOT_ASSIGNED && !ready_queue.empty()) {
            
            //get the item that has the smallest PID and set that as running
            priority_sort(ready_queue);
            
            running = ready_queue.back();
            ready_queue.pop_back();
            running.state = RUNNING;

            if(running.start_time == -1) {
                running.start_time = current_time;
                sync_queue(job_list, running);
            }

            execution_status += print_exec_status(current_time, running.PID, READY, RUNNING);
        }
        
        // Track wait time for processes in ready queue
        for(auto &process : ready_queue) {
            if(process.state == READY) {
                process.wait_time++;
                sync_queue(job_list, process);
            }
        }
        /////////////////////////////////////////////////////////////////

        current_time++;
    }
    
    //Close the output table
    execution_status += print_exec_footer();

    // Calculate metrics
    double total_TAT = 0;
    double total_WT = 0;
    double total_RT = 0;
    int completed_processes = 0;

    for (const auto &p : job_list) {
        if(p.completion_time != -1) {
            int TAT = p.completion_time - p.arrival_time;
            int RT = p.start_time - p.arrival_time;

            total_TAT += TAT;
            total_WT += p.wait_time;
            total_RT += RT;
            completed_processes++;
        }
    }

    double avg_TAT = total_TAT / completed_processes;
    double avg_WT = total_WT / completed_processes;
    double avg_RT = total_RT / completed_processes;
    double throughput = static_cast<double>(completed_processes) / (current_time - 1);

    std::cout << "\n==== Simulation Metrics ====\n";
    std::cout << "Total Processes: " << completed_processes << std::endl;
    std::cout << "Throughput: " << throughput << " processes/unit time" << std::endl;
    std::cout << "Average Turnaround Time: " << avg_TAT << std::endl;
    std::cout << "Average Waiting Time: " << avg_WT << std::endl;
    std::cout << "Average Response Time: " << avg_RT << std::endl;

    return std::make_tuple(execution_status);
}


int main(int argc, char** argv) {

    //Get the input file from the user
    if(argc != 2) {
        std::cout << "ERROR!\nExpected 1 argument, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrupts <your_input_file.txt>" << std::endl;
        return -1;
    }

    //Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    //Ensure that the file actually opens
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    //Parse the entire input file and populate a vector of PCBs.
    //To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;

    while(std::getline(input_file, line)) {
        if(line.empty() || line[0] == '#') continue;
        
        auto input_tokens = split_delim(line, ", ");
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    //With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process);

    // Extract filename from input path for dynamic output naming
    std::string input_path = file_name;
    size_t last_slash = input_path.find_last_of("/\\");
    std::string filename = (last_slash != std::string::npos) ? input_path.substr(last_slash + 1) : input_path;
    
    // Remove .txt extension
    size_t last_dot = filename.find_last_of(".");
    std::string base_name = (last_dot != std::string::npos) ? filename.substr(0, last_dot) : filename;
    
    // Create output path in EP/Outputs folder
    std::string output_path = "EP/Outputs/execution_" + base_name + ".txt";
    
    write_output(exec, output_path.c_str());

    return 0;
}