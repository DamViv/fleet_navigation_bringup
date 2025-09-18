import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import threading
import time

class GotoRemaps(Node):
    def __init__(self):
        super().__init__('goto_remaps')


        # Récupérer le namespace du node pour construire les topics
        namespace = self.get_namespace()        
        self.get_logger().info(f'Starting goto_remaps for namespace: {namespace}')


        # Action client NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose,  f'{namespace}/navigate_to_pose')

        # Subscribe au goal global
        self.goal_sub = self.create_subscription(PoseStamped, f'{namespace}/goal_pose_', self.goal_callback, 10)
        self.pending_goal = None
        self.current_goal_handle = None
        self.goal_lock = threading.Lock()

        self.waypoints_sub = self.create_subscription(PoseArray, f'{namespace}/waypoints', self.waypoints_callback, 10)
        self.pending_waypoints = []   
        self.current_waypoint_index = 0
      
        self.should_cancel = False
        
        
        # Événement pour signaler un nouveau goal
        self.new_goal_event = threading.Event()
        
        # Démarrer le thread de navigation permanent
        self.nav_thread = threading.Thread(target=self.navigation_loop)
        self.nav_thread.daemon = True
        self.nav_thread.start()

    def goal_callback(self, msg):
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        
        with self.goal_lock:            
            self.should_cancel = True
            if self.current_goal_handle is not None:
                self.get_logger().info('Signaling cancellation of current goal...')
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().warn(f'Failed to signal goal cancellation: {e}')

            # Stocker le nouveau goal
            self.pending_waypoints = [msg]
            self.current_waypoint_index = 0
            self.should_cancel = False
            
        # Signaler au thread de navigation qu'un nouveau goal est disponible
        self.new_goal_event.set()
        self.get_logger().info('Goal stored and navigation thread notified')

    def waypoints_callback(self, msg):
        """Callback pour une liste de waypoints"""
        self.get_logger().info(f'Waypoints received: {len(msg.poses)} points')
        
        with self.goal_lock:
            self.should_cancel = True
            
            if self.current_goal_handle is not None:
                self.get_logger().info('Cancelling current navigation...')
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().warn(f'Failed to cancel goal: {e}')

            # Convertir PoseArray en liste de PoseStamped
            self.pending_waypoints = []
            for i, pose in enumerate(msg.poses):
                waypoint = PoseStamped()
                waypoint.header.frame_id = msg.header.frame_id or 'map'
                waypoint.header.stamp = self.get_clock().now().to_msg()
                waypoint.pose = pose
                self.pending_waypoints.append(waypoint)
                self.get_logger().info(f'  Waypoint {i+1}: ({pose.position.x:.2f}, {pose.position.y:.2f})')
            
            self.current_waypoint_index = 0
            self.should_cancel = False
            
        self.new_goal_event.set()
        self.get_logger().info(f'Waypoint list stored: {len(self.pending_waypoints)} points')



    def navigation_loop(self):
        """Thread permanent qui traite les goals"""
        while rclpy.ok():
            try:
                # Attendre qu'un goal soit disponible ou timeout de 1 seconde
                self.new_goal_event.wait(timeout=1.0)
                self.new_goal_event.clear()
                
                with self.goal_lock:
                    waypoints = self.pending_waypoints.copy()
                    current_index = self.current_waypoint_index
                    should_cancel = self.should_cancel
                
                if waypoints and current_index < len(waypoints) and not should_cancel:
                    self.get_logger().info(f'Processing waypoint sequence: {len(waypoints)} waypoints, starting from {current_index + 1}')
                    
                    # Traiter tous les waypoints séquentiellement
                    success = self.process_waypoint_sequence(waypoints, current_index)
                    
                    with self.goal_lock:
                        if self.pending_waypoints == waypoints and not self.should_cancel:
                            if success:
                                self.get_logger().info('All waypoints completed successfully!')
                            else:
                                self.get_logger().warn('Waypoint sequence failed or was cancelled')
                            self.pending_waypoints = []
                            self.current_waypoint_index = 0
                            self.current_goal_handle = None
                        else:
                            self.get_logger().info('Waypoint sequence was replaced during navigation')
                
            except Exception as e:
                self.get_logger().error(f'Error in navigation loop: {e}')
                time.sleep(1.0)


    def process_waypoint_sequence(self, waypoints, start_index):
        """Traite une séquence de waypoints"""
        for i in range(start_index, len(waypoints)):
            with self.goal_lock:
                if self.should_cancel or self.pending_waypoints != waypoints:
                    self.get_logger().info('Waypoint sequence cancelled by new goals')
                    return False
                self.current_waypoint_index = i
            
            current_waypoint = waypoints[i]
            self.get_logger().info(f'Navigating to waypoint {i+1}/{len(waypoints)}: ({current_waypoint.pose.position.x:.2f}, {current_waypoint.pose.position.y:.2f})')
            
            # Utiliser la fonction send_waypoint existante
            success = self.send_waypoint(current_waypoint)
            
            if not success:
                self.get_logger().warn(f'Failed to reach waypoint {i+1}, stopping sequence')
                return False
            
            self.get_logger().info(f'Waypoint {i+1}/{len(waypoints)} reached successfully')
            
            # Petite pause entre les waypoints pour éviter la surcharge
            time.sleep(0.1)
        
        return True

    def send_waypoint(self, pose_stamped):
        """Envoie un waypoint et retourne True si atteint avec succès"""
        with self.goal_lock:
            if self.should_cancel:
                return False
        
        self.get_logger().info(f'Sending waypoint to Nav2: ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})')
            
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Navigation server not available')
            return False
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        
        # Envoyer le goal de manière asynchrone
        future = self.nav_client.send_goal_async(goal_msg)
        
        # Attendre que le goal soit accepté en polling
        timeout = 0
        while not future.done() and timeout < 50:  # 5 secondes max
            time.sleep(0.1)
            timeout += 1
            with self.goal_lock:
                if self.should_cancel:
                    self.get_logger().info('Goal cancelled during send')
                    return False
        
        if not future.done():
            self.get_logger().warn('Goal send timeout')
            return False
            
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint rejected by Nav2')
            return False
        
        self.get_logger().info('Waypoint accepted by Nav2, waiting for completion...')
        
        with self.goal_lock:
            self.current_goal_handle = goal_handle

        # Attendre le résultat en polling
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            with self.goal_lock:
                if self.should_cancel:
                    self.get_logger().info('Waypoint cancelled by new goal')
                    # Annuler le goal en cours
                    try:
                        cancel_future = goal_handle.cancel_goal_async()
                        # Attendre l'annulation en polling
                        cancel_timeout = 0
                        while not cancel_future.done() and cancel_timeout < 10:
                            time.sleep(0.1)
                            cancel_timeout += 1
                    except Exception as e:
                        self.get_logger().warn(f'Failed to cancel waypoint: {e}')
                    with self.goal_lock:
                        self.current_goal_handle = None
                    return False
            
            time.sleep(0.1)  # Polling toutes les 100ms
            
        result = result_future.result()
        
        with self.goal_lock:
            self.current_goal_handle = None
        
        self.get_logger().info(f'Nav2 result received for waypoint ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})')
        
        # Vérifier le statut du résultat
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✓ Waypoint reached: ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})')
            return True
        else:
            status_names = {
                1: "EXECUTING",
                2: "CANCELING", 
                3: "SUCCEEDED",
                4: "CANCELED",
                5: "ABORTED"
            }
            status_name = status_names.get(result.status, f"UNKNOWN({result.status})") if result else "NO_RESULT"
            self.get_logger().warn(f'✗ Failed to reach waypoint: ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f}) - Status: {status_name}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = GotoRemaps()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
