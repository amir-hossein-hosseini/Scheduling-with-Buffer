def Extract_Parameters(file_path):
    with open(file_path, 'r') as file:
        lines = [line.strip() for line in file.readlines() if line.strip()]
    
    # Extract number of jobs (n) and machines (m)
    n, m = 0, 0
    for i, line in enumerate(lines):
        if "number of jobs" in line:
            parts = lines[i+1].split()
            n = int(parts[0])
            m = int(parts[1])
            break
    
    # Extract processing times
    processing_times = []
    found_processing = False
    for line in lines:
        if "processing times:" in line:
            found_processing = True
            continue
        if found_processing and not line[0].isalpha():  # Skip non-data lines
            if not processing_times:
                # First machine's processing times
                processing_times = [[int(num)] for num in line.split()]
            else:
                # Subsequent machines' processing times
                nums = list(map(int, line.split()))
                for i in range(len(processing_times)):
                    processing_times[i].append(nums[i])
                    if len(processing_times[i]) == m:
                        processing_times[i] = processing_times[i][:m]
            if len(processing_times) >= n and all(len(pt) == m for pt in processing_times):
                break
    
    # Extract transportation times
    transportation_times = []
    found_transportation = False
    for line in lines:
        if "transportation times:" in line:
            found_transportation = True
            continue
        if found_transportation and not line[0].isalpha():  # Skip non-data lines
            if len(transportation_times) < m + 2:
                transportation_times.append(list(map(int, line.split())))
            if len(transportation_times) == m + 2:
                break
    
    # Extract input buffer
    input_buffer = []
    found_input = False
    for line in lines:
        if "input buffer:" in line:
            found_input = True
            continue
        if found_input and not line[0].isalpha():  # Skip non-data lines
            input_buffer = list(map(int, line.split()))
            if len(input_buffer) == m:
                break
    
    # Extract output buffer
    output_buffer = []
    found_output = False
    for line in lines:
        if "output buffer:" in line:
            found_output = True
            continue
        if found_output and not line[0].isalpha():  # Skip non-data lines
            output_buffer = list(map(int, line.split()))
            if len(output_buffer) == m:
                break
    
    return {
        "num_jobs": n,
        "num_machines": m,
        "processing_times": processing_times,
        "transportation_times": transportation_times,
        "input_buffer": input_buffer,
        "output_buffer": output_buffer
    }