import pygame
import asyncio
import math
import random

# You can load this onto localhost using pygbag (pip install pygbag) and then run the following line:
# pygbag --template main.tmpl main.py
# This creates the build which was uploaded to itch.io for easier playing

# The main.tmpl file removes the default pygame start-up screen which looked very ugly. 
# It's implementation can be found here (I did not make it): https://github.com/timetrapped-creations/pygbag 

WIDTH, HEIGHT = 1000, 600
PLAYER_SPEED = 5 
ENEMY_SPEED = 3
OBSTACLE_COUNT = 8
COLLECTIBLE_COUNT = 5

# Colors
WHITE = (255, 255, 255)
BLACK = (20, 20, 20)
RED = (200, 50, 50)
GREEN = (50, 200, 50)
BLUE = (50, 100, 200)
GRAY = (100, 100, 100)
YELLOW = (200, 200, 50)
CYAN = (50, 200, 200)
ORANGE = (255, 165, 0)

class Player(pygame.sprite.Sprite):
    def __init__(self, x, y, obstacles):
        super().__init__()
        self.image = pygame.Surface((30, 30))
        self.image.fill(GREEN)
        self.rect = self.image.get_rect(center=(x, y))
        self.pos = pygame.math.Vector2(x, y)
        self.obstacles = obstacles

    def is_collision_free(self, test_pos):
        """Check if a position is free of obstacles"""
        test_rect = pygame.Rect(test_pos.x - 15, test_pos.y - 15, 30, 30)
        for obs in self.obstacles:
            if test_rect.colliderect(obs.rect):
                return False
        return True

    def update(self, player_speed):
        keys = pygame.key.get_pressed()
        move = pygame.math.Vector2(0, 0)

        if keys[pygame.K_w] or keys[pygame.K_UP]: move.y = -1
        if keys[pygame.K_s] or keys[pygame.K_DOWN]: move.y = 1
        if keys[pygame.K_a] or keys[pygame.K_LEFT]: move.x = -1
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]: move.x = 1

        if move.length() > 0:
            move = move.normalize() * player_speed
            new_pos = self.pos + move

            new_pos.x = max(15, min(WIDTH - 15, new_pos.x))
            new_pos.y = max(15, min(HEIGHT - 15, new_pos.y))

            # Only move if collision-free
            if self.is_collision_free(new_pos):
                self.pos = new_pos

            self.rect.center = round(self.pos.x), round(self.pos.y)

class Obstacle(pygame.sprite.Sprite):
    def __init__(self, x, y, w, h):
        super().__init__()
        self.image = pygame.Surface((w, h))
        self.image.fill(GRAY)
        self.rect = self.image.get_rect(topleft=(x, y))

class Collectible(pygame.sprite.Sprite):
    def __init__(self, x, y):
        super().__init__()
        self.image = pygame.Surface((15, 15))
        self.image.fill(CYAN)
        self.rect = self.image.get_rect(center=(x, y))
        self.pos = pygame.math.Vector2(x, y)

# RRT* algorithm starts here. Sorry about the magic numbers

class RRTNode:
    def __init__(self, pos):
        self.pos = pygame.math.Vector2(pos)
        self.parent = None
        self.children = []
        self.cost = 0  

class RRTStar:
    def __init__(self, start_pos, obstacles, max_distance=80, goal_bias=0.3):
        self.root = RRTNode(start_pos)
        self.root.cost = 0
        self.nodes = [self.root]
        self.obstacles = obstacles
        self.max_distance = max_distance
        self.goal_bias = goal_bias

    def is_position_valid(self, pos):
        test_rect = pygame.Rect(pos.x - 15, pos.y - 15, 30, 30)
        for obs in self.obstacles:
            if test_rect.colliderect(obs.rect):
                return False
        return True

    def is_path_clear(self, start_pos, end_pos, steps=10):
        for i in range(steps):
            t = i / steps
            check_pos = start_pos + (end_pos - start_pos) * t
            if not self.is_position_valid(check_pos):
                return False
        return True

    def get_nearby_nodes(self, pos, radius=150):
        return [n for n in self.nodes if n.pos.distance_to(pos) < radius]

    # I think that making this a * algorithm was a bit unnecessary since we move too quickly for rewiring to 
    # show significant effects but I did notice that this made my paths significantly faster. Much of this is 
    # based on my assignment code
    def rewire(self, new_node, nearby_nodes):
        for node in nearby_nodes:
            new_cost = new_node.cost + new_node.pos.distance_to(node.pos)
            if new_cost < node.cost and self.is_path_clear(new_node.pos, node.pos):

                if node.parent:
                    node.parent.children.remove(node)
                node.parent = new_node
                new_node.children.append(node)
                node.cost = new_cost

                self._update_descendant_costs(node)

    def _update_descendant_costs(self, node):
        for child in node.children:
            child.cost = node.cost + node.pos.distance_to(child.pos)
            self._update_descendant_costs(child)

    def extend(self, target_pos, max_iterations=1):
        for _ in range(max_iterations):
            if random.random() < self.goal_bias:
                sample = pygame.math.Vector2(target_pos)
            else:
                sample = pygame.math.Vector2(
                    random.uniform(0, WIDTH),
                    random.uniform(0, HEIGHT)
                )

            nearest_node = min(self.nodes, key=lambda n: n.pos.distance_to(sample))
            
            direction = sample - nearest_node.pos
            if direction.length() > 0:
                direction = direction.normalize()
            
            step = min(self.max_distance, (sample - nearest_node.pos).length())
            new_pos = nearest_node.pos + direction * step
            
            new_pos.x = max(15, min(WIDTH - 15, new_pos.x))
            new_pos.y = max(15, min(HEIGHT - 15, new_pos.y))

            if self.is_path_clear(nearest_node.pos, new_pos) and self.is_position_valid(new_pos):
                new_node = RRTNode(new_pos)
                new_node.cost = nearest_node.cost + nearest_node.pos.distance_to(new_pos)
                new_node.parent = nearest_node
                nearest_node.children.append(new_node)
                self.nodes.append(new_node)
                
                nearby_nodes = self.get_nearby_nodes(new_pos)
                self.rewire(new_node, nearby_nodes)

                if new_pos.distance_to(target_pos) < self.max_distance:
                    if self.is_path_clear(new_pos, target_pos) and self.is_position_valid(target_pos):
                        goal_node = RRTNode(target_pos)
                        goal_node.cost = new_node.cost + new_node.pos.distance_to(target_pos)
                        goal_node.parent = new_node
                        new_node.children.append(goal_node)
                        self.nodes.append(goal_node)
                        return self.get_path(goal_node)

        return None

    def get_path(self, node):
        path = []
        current = node
        while current is not None:
            path.append(pygame.math.Vector2(current.pos))
            current = current.parent
        path.reverse()
        return path

    def get_tree_edges(self):
        edges = []
        for node in self.nodes:
            if node.parent:
                edges.append((node.parent.pos, node.pos))
        return edges

# This enemy class movement is based on another project that I was working on. Adapted to fix this. 
class Enemy(pygame.sprite.Sprite):
    def __init__(self, x, y, obstacles):
        super().__init__()
        self.image = pygame.Surface((30, 30))
        self.image.fill(RED)
        self.rect = self.image.get_rect(center=(x, y))
        self.pos = pygame.math.Vector2(x, y)
        self.vel = pygame.math.Vector2(0, 0)
        self.obstacles = obstacles
        
        # RRT* variables are defined here. A higher bias makes sense for this environment.
        # From trial and error, lower bias trees tended to loop strangely if you were far from them.
        self.rrt = RRTStar(self.pos, obstacles, max_distance=80, goal_bias=0.35)
        self.path = []
        self.path_index = 0
        self.replan_cooldown = 0
        self.REPLAN_RATE = 15
        
        # Handles niche edge cases where the square gets stuck in a weird position. The functions for that are directly below this. 
        self.last_pos = pygame.math.Vector2(x, y)
        self.stuck_counter = 0
        self.is_recovering = False
        self.recovery_timer = 0
        self.recovery_dir = pygame.math.Vector2(0, 0)
        self.collision_avoidance_dir = None

    def is_collision_free(self, test_pos):
        if (test_pos.x < 15 or test_pos.x > WIDTH - 15 or 
            test_pos.y < 15 or test_pos.y > HEIGHT - 15):
            return False

        test_rect = pygame.Rect(test_pos.x - 15, test_pos.y - 15, 30, 30)
        for obs in self.obstacles:
            if test_rect.colliderect(obs.rect):
                return False
        return True

    def get_random_recovery_dir(self):
        angle = random.uniform(0, 2 * math.pi)
        return pygame.math.Vector2(math.cos(angle), math.sin(angle))

    def find_collision_avoidance_direction(self):
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        for angle_deg in angles:
            angle_rad = math.radians(angle_deg)
            direction = pygame.math.Vector2(math.cos(angle_rad), math.sin(angle_rad))
            test_pos = self.pos + direction * 40
            if self.is_collision_free(test_pos):
                return direction
        return None

    def update(self, player_pos, enemy_speed):
        if self.pos.distance_to(self.last_pos) < 1.0 and not self.is_recovering:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
            
        self.last_pos = pygame.math.Vector2(self.pos)

        if self.stuck_counter > 18:
            self.is_recovering = True
            self.recovery_timer = 25
            self.recovery_dir = self.get_random_recovery_dir()
            self.stuck_counter = 0
            self.image.fill(ORANGE)

        if self.is_recovering:
            self.recovery_timer -= 1
            if self.recovery_timer <= 0:
                self.is_recovering = False
                self.image.fill(RED)
                self.rrt = RRTStar(self.pos, self.obstacles, max_distance=80, goal_bias=0.35)
                self.path = []
                self.collision_avoidance_dir = None
            else:
                new_pos = self.pos + self.recovery_dir * (enemy_speed * 0.8)
                if self.is_collision_free(new_pos):
                    self.pos = new_pos
                else:
                    self.recovery_dir = self.get_random_recovery_dir()
            
            self.rect.center = round(self.pos.x), round(self.pos.y)
            return

        self.replan_cooldown -= 1
        if self.replan_cooldown <= 0 or not self.path:
            self.rrt = RRTStar(self.pos, self.obstacles, max_distance=80, goal_bias=0.35)
            for _ in range(500): 
                path = self.rrt.extend(player_pos, max_iterations=1)
                if path:
                    self.path = path
                    self.path_index = 0
                    self.collision_avoidance_dir = None
                    break
            self.replan_cooldown = self.REPLAN_RATE

        if self.path and self.path_index < len(self.path):
            target = self.path[self.path_index]
            direction = target - self.pos
            dist = direction.length()

            if dist > enemy_speed:
                direction = direction.normalize()
                new_pos = self.pos + direction * enemy_speed
                
                if self.is_collision_free(new_pos):
                    self.pos = new_pos
                    self.vel = direction * enemy_speed
                    self.collision_avoidance_dir = None
                else:
                    if self.collision_avoidance_dir is None:
                        self.collision_avoidance_dir = self.find_collision_avoidance_direction()
                    
                    if self.collision_avoidance_dir:
                        new_pos = self.pos + self.collision_avoidance_dir * enemy_speed
                        if self.is_collision_free(new_pos):
                            self.pos = new_pos
                        else:
                            self.collision_avoidance_dir = None
                    else:
                        self.replan_cooldown = 0
            else:
                self.pos = target
                self.path_index += 1
        
        self.rect.center = round(self.pos.x), round(self.pos.y)

async def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("RRT* Chase - Optimized Pathfinding")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)
    title_font = pygame.font.Font(None, 48)

    screen.fill(BLACK)
    pygame.display.flip()

    def spawn_collectibles(collectibles, all_sprites, obstacles):
        for _ in range(COLLECTIBLE_COUNT):
            attempts = 0
            while attempts < 100:
                x = random.randint(50, WIDTH - 50)
                y = random.randint(50, HEIGHT - 50)
                test_rect = pygame.Rect(x - 10, y - 10, 20, 20)
                valid = True
                for obs in obstacles:
                    if test_rect.colliderect(obs.rect):
                        valid = False
                        break
                if valid:
                    for col in collectibles:
                        if math.hypot(x - col.pos.x, y - col.pos.y) < 80:
                            valid = False
                            break
                if valid:
                    col = Collectible(x, y)
                    collectibles.add(col)
                    all_sprites.add(col)
                    break
                attempts += 1

    game_state = "menu"
    show_tree = False
    score = 0
    last_spawn_score = 0

    # Claude, "Please give me boilerplate of a pygame introduction menu"
    # Usually for my projects I just hand draw these but that didn't make sense here. 

    def draw_menu():
        screen.fill(BLACK)

        title_text = title_font.render("RRT* CHASE", True, CYAN)
        title_rect = title_text.get_rect(center=(WIDTH // 2, 120))
        screen.blit(title_text, title_rect)

        subtitle = font.render("Optimized Pathfinding AI. By: Sonam Wangchuk '26", True, BLUE)
        subtitle_rect = subtitle.get_rect(center=(WIDTH // 2, 180))
        screen.blit(subtitle, subtitle_rect)

        instructions = [
            "WASD or Arrow Keys - Move",
            "T - Toggle to see the RRT* Paths",
            "Collect cyan dots to score",
            "Avoid the red enemy!",
            "",
            "Press SPACE to Start"
        ]
        
        y_offset = 280
        for line in instructions:
            if line == "":
                y_offset += 10
                continue
            text = font.render(line, True, WHITE if "SPACE" not in line else GREEN)
            text_rect = text.get_rect(center=(WIDTH // 2, y_offset))
            screen.blit(text, text_rect)
            y_offset += 35
        
        footer = pygame.font.Font(None, 18).render("Enemy gets faster as you score!", True, GRAY)
        screen.blit(footer, (WIDTH // 2 - footer.get_width() // 2, HEIGHT - 40))
        
        pygame.display.flip()

    def setup_game():
        nonlocal score, last_spawn_score
        score = 0
        last_spawn_score = 0
        
        all_sprites = pygame.sprite.Group()
        obstacles = pygame.sprite.Group()
        collectibles = pygame.sprite.Group()

        placed_obstacles = []
        attempts = 0
        max_attempts = 500
        
        while len(placed_obstacles) < OBSTACLE_COUNT and attempts < max_attempts:
            w, h = random.randint(60, 120), random.randint(60, 120)
            x = random.randint(20, WIDTH - w - 20)
            y = random.randint(20, HEIGHT - h - 20)
            new_rect = pygame.Rect(x, y, w, h)
            
            if (math.hypot(x + w/2 - WIDTH//2, y + h/2 - HEIGHT//2) > 200 and 
                math.hypot(x + w/2 - 50, y + h/2 - 50) > 200):
                overlap = False
                for existing_rect in placed_obstacles:
                    if new_rect.colliderect(existing_rect.inflate(30, 30)):
                        overlap = True
                        break
                if not overlap:
                    obs = Obstacle(x, y, w, h)
                    obstacles.add(obs)
                    all_sprites.add(obs)
                    placed_obstacles.append(new_rect)
            attempts += 1

        player = Player(WIDTH // 2, HEIGHT // 2, obstacles)
        all_sprites.add(player)

        enemy = Enemy(50, 50, obstacles)
        all_sprites.add(enemy)

        spawn_collectibles(collectibles, all_sprites, obstacles)
        return all_sprites, obstacles, collectibles, player, enemy

    all_sprites, obstacles, collectibles, player, enemy = setup_game()

    # The following is based on publicly available pygame UI boilerplate

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_t:
                    show_tree = not show_tree
                if event.key == pygame.K_r and game_state == "caught":
                    game_state = "playing"
                    all_sprites, obstacles, collectibles, player, enemy = setup_game()
                if event.key == pygame.K_SPACE and game_state == "menu":
                    game_state = "playing"
                    all_sprites, obstacles, collectibles, player, enemy = setup_game()

        if game_state == "menu":
            draw_menu()
        elif game_state == "playing":
            current_player_speed = PLAYER_SPEED + (score // 5) * 0.25
            current_enemy_speed = ENEMY_SPEED + (score // 5) * 0.5
            
            player.update(current_player_speed)
            enemy.update(player.pos, current_enemy_speed)

            hit_collectibles = pygame.sprite.spritecollide(player, collectibles, True)
            score += len(hit_collectibles)

            if score >= last_spawn_score + 5:
                spawn_collectibles(collectibles, all_sprites, obstacles)
                last_spawn_score = score

            if player.rect.colliderect(enemy.rect):
                game_state = "caught"

        screen.fill(BLACK)

        if game_state == "menu":
            draw_menu()
        elif game_state == "playing":
            if show_tree:
                for edge in enemy.rrt.get_tree_edges():
                    pygame.draw.line(screen, (50, 50, 150), edge[0], edge[1], 1)
                for node in enemy.rrt.nodes:
                    pygame.draw.circle(screen, (0, 0, 255), (int(node.pos.x), int(node.pos.y)), 2)

            if show_tree and enemy.path and not enemy.is_recovering:
                for i in range(len(enemy.path) - 1):
                    pygame.draw.line(screen, YELLOW, enemy.path[i], enemy.path[i+1], 3)

            if show_tree:
                pygame.draw.circle(screen, GREEN, (int(player.pos.x), int(player.pos.y)), 8, 2)
                color = ORANGE if enemy.is_recovering else RED
                pygame.draw.circle(screen, color, (int(enemy.pos.x), int(enemy.pos.y)), 8, 2)

            all_sprites.draw(screen)

            status_text = "Status: RECOVERING" if enemy.is_recovering else "Status: CHASING"
            info_text = font.render(f"Score: {score} | {status_text} | Press T to see RRT*", True, WHITE)
            screen.blit(info_text, (10, 10))

        else:
            game_over_text = title_font.render("CAUGHT!", True, RED)
            score_text = font.render(f"Final Score: {score}", True, WHITE)
            restart_text = font.render("Press R to Restart", True, WHITE)
            
            screen.blit(game_over_text, (WIDTH // 2 - game_over_text.get_width() // 2, HEIGHT // 2 - 100))
            screen.blit(score_text, (WIDTH // 2 - score_text.get_width() // 2, HEIGHT // 2 - 20))
            screen.blit(restart_text, (WIDTH // 2 - restart_text.get_width() // 2, HEIGHT // 2 + 40))

        pygame.display.flip()
        clock.tick(60)

        await asyncio.sleep(0)

if __name__ == "__main__":
    asyncio.run(main())