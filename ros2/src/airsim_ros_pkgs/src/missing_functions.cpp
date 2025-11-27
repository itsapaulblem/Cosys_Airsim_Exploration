void VehicleNodeBase::lidar_timer_callback()
{
    // CRITICAL: Initialization guard - prevent execution before RPC clients ready
    if (!initialization_complete_.load()) {
        return;
    }

    // CRITICAL: Non-blocking guard to prevent concurrent callback execution
    std::unique_lock<std::mutex> lock(lidar_callback_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        return;
    }

    try {
        process_lidar();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "lidar processing");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "LiDAR processing error: %s", e.what());
    }
}

void VehicleNodeBase::gpulidar_timer_callback()
{
    // CRITICAL: Initialization guard - prevent execution before RPC clients ready
    if (!initialization_complete_.load()) {
        return;
    }

    try {
        process_gpulidar();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "gpulidar processing");
    }
}

// Individual camera timer callbacks for parallel processing
void VehicleNodeBase::camera_1_timer_callback()
{
    try {
        process_camera_1();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "camera 1 processing");
    }
}

void VehicleNodeBase::camera_2_timer_callback()
{
    try {
        process_camera_2();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "camera 2 processing");
    }
}

void VehicleNodeBase::camera_3_timer_callback()
{
    try {
        process_camera_3();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "camera 3 processing");
    }
}

void VehicleNodeBase::camera_4_timer_callback()
{
    try {
        process_camera_4();
    }
    catch (const rpc::rpc_error& e) {
        handle_rpc_error(e, "camera 4 processing");
    }
}