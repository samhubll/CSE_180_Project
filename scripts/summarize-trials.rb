#!/usr/bin/env ruby
# Offline validation analysis, separate from the C++ search algorithm.
require 'json'
require 'csv'
root = File.expand_path('..', __dir__)
logs = File.join(root, 'setup-logs/milestone3')
pose = lambda do |path|
  return nil unless File.file?(path)
  arrays = File.read(path).scan(/\[([-+0-9.eE ]+)\]/).map { |match| match[0].split.map(&:to_f) }.select { |a| a.size == 3 }
  arrays.size >= 2 ? { xyz: arrays[0], rpy: arrays[1] } : nil
end
wrap = ->(a) { Math.atan2(Math.sin(a), Math.cos(a)) }
# Validation-only projection from measured stationary Gazebo pose. Recognition
# labels must not depend on AMCL being accurate enough to localize the person.
camera_geometry = lambda do |robot, target|
  roll, pitch, yaw = robot[:rpy]
  cr, sr, cp, sp, cy, sy = Math.cos(roll), Math.sin(roll), Math.cos(pitch), Math.sin(pitch), Math.cos(yaw), Math.sin(yaw)
  rotation = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
  mount = [-0.0596, 0.0, 0.24353]
  camera = 3.times.map { |i| robot[:xyz][i] + 3.times.sum { |j| rotation[i][j] * mount[j] } }
  delta = 3.times.map { |i| target[:xyz][i] + (i == 2 ? 1.0 : 0.0) - camera[i] }
  base_ray = 3.times.map { |i| 3.times.sum { |j| rotation[j][i] * delta[j] } }
  optical = [-base_ray[1], -base_ray[2], base_ray[0]]
  uv = optical[2] > 0 ? [221.76500407999384 * optical[0] / optical[2] + 160.0, 221.76500407999382 * optical[1] / optical[2] + 120.0] : nil
  dx, dy = target[:xyz][0] - camera[0], target[:xyz][1] - camera[1]
  { xyz: camera, uv: uv, optical_depth_m: optical[2], horizontal_distance_m: Math.hypot(dx, dy), bearing_rad: wrap.call(Math.atan2(dy, dx) - yaw), aspect_rad: wrap.call(Math.atan2(-dy, -dx) - target[:rpy][2]) }
end
stats = lambda do |values|
  values = values.compact.sort
  next nil if values.empty?
  n = values.size
  { min: values.first, median: n.odd? ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2.0, max: values.last }
end
accepted = Hash.new { |h, k| h[k] = [] }
rejected = Hash.new { |h, k| h[k] = [] }
Dir[File.join(logs, 'search*.log')].sort.each do |path|
  File.foreach(path) do |line|
    if (m = /Observation source=(\d+)\.(\d{9}).*map=\(([-+\d.]+),([-+\d.]+),([-+\d.]+)\).*id=(\d+) images=(\d+) confirmed=(yes|no)/.match(line))
      ns = m[1].to_i * 1_000_000_000 + m[2].to_i
      accepted[ns] << { xyz: m[3..5].map(&:to_f), id: m[6].to_i, images: m[7].to_i, confirmed: m[8] == 'yes', log: path }
    elsif (m = /Localization rejection source=(\d+)\.(\d{9}) reason=(.*)$/.match(line))
      rejected[m[1].to_i * 1_000_000_000 + m[2].to_i] << m[3]
    end
  end
end
performance = {}
Dir[File.join(logs, 'detector*.log')].sort.each do |path|
  File.foreach(path) do |line|
    next unless line.include?('PERFORMANCE ')
    entry = JSON.parse(line.split('PERFORMANCE ', 2)[1])
    performance[entry['source_ns']] = entry
  end
end
summaries = Dir[File.join(logs, 'trials/*/images.jsonl')].sort.filter_map do |path|
  rows = File.readlines(path).reject { |line| line.strip.empty? }.map { |line| JSON.parse(line) }
  summary = rows.reverse.find { |row| row['type'] == 'trial_summary' }
  next unless summary
  images = rows.select { |row| row['type'] == 'image' }
  dir = File.dirname(path)
  before = pose.call(File.join(dir, 'robot-before.txt'))
  after = pose.call(File.join(dir, 'robot-after.txt'))
  target = pose.call(File.join(dir, 'target-before.txt'))
  truth = target && target[:xyz]
  actual_accepted = images.count { |row| !accepted[row['source_ns']].empty? }
  accurate = images.count do |row|
    truth && accepted[row['source_ns']].any? { |entry| Math.hypot(entry[:xyz][0] - truth[0], entry[:xyz][1] - truth[1]) <= 0.5 }
  end
  reasons = images.flat_map { |row| rejected[row['source_ns']] }.tally
  perf = images.filter_map { |row| performance[row['source_ns']] }
  confidences = images.flat_map { |row| row['boxes'].flat_map { |box| box['hypotheses'].select { |h| h['class'] == 'person' }.map { |h| h['confidence'] } } }
  localization_errors = images.flat_map do |row|
    next [] unless truth
    accepted[row['source_ns']].map { |entry| Math.hypot(entry[:xyz][0] - truth[0], entry[:xyz][1] - truth[1]) }
  end
  result = summary.merge(
    'actual_search_accepted_images' => actual_accepted,
    'actual_search_accurate_images' => accurate,
    'actual_search_rejections' => reasons,
    'actual_search_error_m' => stats.call(localization_errors),
    'actual_search_confirmed_ids' => images.flat_map { |row| accepted[row['source_ns']].select { |entry| entry[:confirmed] }.map { |entry| entry[:id] } }.uniq,
    'actual_search_ids' => images.flat_map { |row| accepted[row['source_ns']].map { |entry| entry[:id] } }.uniq,
    'confidence' => stats.call(confidences),
    'prediction_wall_ms' => stats.call(perf.map { |entry| entry['prediction_wall_ms'] }),
    'performance_images' => perf.size,
    'thresholds' => perf.map { |entry| entry['confidence_threshold'] }.uniq,
    'actual_threads' => perf.map { |entry| entry['cpu_threads_actual'] }.uniq,
    'inference_shapes' => perf.map { |entry| entry['inference_shape_nchw'] }.uniq,
    'robot_before' => before, 'robot_after' => after, 'target_pose' => target,
    'source_header_matches' => images.count { |row| row['source_rgb_seen'] },
    'source_box_header_mismatches' => images.sum { |row| row['boxes'].count { |box| !box['header_matches'] } }
  )
  if before && truth
    dx, dy = truth[0] - before[:xyz][0], truth[1] - before[:xyz][1]
    result['gazebo_robot_horizontal_distance_m'] = Math.hypot(dx, dy)
    result['gazebo_robot_bearing_rad'] = wrap.call(Math.atan2(dy, dx) - before[:rpy][2])
    result['gazebo_person_aspect_rad'] = wrap.call(Math.atan2(-dy, -dx) - target[:rpy][2])
    geometry = camera_geometry.call(before, target)
    result['gazebo_camera'] = geometry
    recognized_sources = images.filter_map do |row|
      uv = geometry[:uv]
      row['source_ns'] if uv && row['boxes'].any? { |box| (uv[0] - box['cx']).abs <= box['width'] / 2.0 && (uv[1] - box['cy']).abs <= box['height'] / 2.0 }
    end
    result['gazebo_projected_target_detection_images'] = recognized_sources.size
    result['recognized_images'] = result['gazebo_projected_target_detection_images']
    result['actual_usable_images'] = images.count do |row|
      recognized_sources.include?(row['source_ns']) && accepted[row['source_ns']].any? { |entry| Math.hypot(entry[:xyz][0] - truth[0], entry[:xyz][1] - truth[1]) <= 0.5 }
    end
  end
  if before && after
    result['gazebo_robot_motion_m'] = Math.hypot(before[:xyz][0] - after[:xyz][0], before[:xyz][1] - after[:xyz][1])
    result['gazebo_robot_yaw_change_rad'] = wrap.call(after[:rpy][2] - before[:rpy][2])
  end
  annotations_path = File.join(dir, 'annotations.json')
  if File.file?(annotations_path)
    result['annotations'] = JSON.parse(File.read(annotations_path))
    result['recognized_images'] = result['annotations']['manual_recognized_images'] if result['annotations'].key?('manual_recognized_images')
    result['actual_usable_images'] = accurate if result['recognized_images'] == images.size
    result['actual_usable_images'] = 0 if result['recognized_images'] == 0
  end
  File.write(File.join(dir, 'correlated-summary.json'), JSON.pretty_generate(result) + "\n")
  result
end
out = ARGV[0] || File.join(logs, 'trial-summaries.json')
File.write(out, JSON.pretty_generate(summaries) + "\n")
csv_path = ARGV[1] || File.join(logs, 'trial-summaries.csv')
CSV.open(csv_path, 'w') do |csv|
  csv << %w[trial threshold processed_images recognized_images actual_search_accepted_images actual_usable_images confirmed_ids gazebo_camera_range_m bearing_deg person_aspect_deg confidence_min confidence_median confidence_max roi_valid_min roi_total_min localization_error_min_m localization_error_max_m actual_rejections source_hz wall_hz latency_min_sim_s latency_median_sim_s latency_max_sim_s prediction_median_wall_ms observed_false_positive_boxes motion_during_trial_m localization_condition]
  summaries.each do |s|
    dir = File.join(logs, 'trials', s['trial'])
    rows = File.readlines(File.join(dir, 'images.jsonl')).map { |line| JSON.parse(line) }.select { |row| row['type'] == 'image' }
    boxes = rows.flat_map { |row| row['boxes'] }
    g = s['gazebo_camera'] || {}
    csv << [s['trial'], s['thresholds'].join('/'), s['distinct_processed_images'], s['recognized_images'], s['actual_search_accepted_images'], s['actual_usable_images'], s['actual_search_confirmed_ids'].join('/'), g[:horizontal_distance_m], g[:bearing_rad] && g[:bearing_rad]*180/Math::PI, g[:aspect_rad] && g[:aspect_rad]*180/Math::PI, s.dig('confidence', :min), s.dig('confidence', :median), s.dig('confidence', :max), boxes.filter_map { |b| b['valid_depth_pixels'] }.min, boxes.filter_map { |b| b['roi_pixels'] }.min, s.dig('actual_search_error_m', :min), s.dig('actual_search_error_m', :max), s['actual_search_rejections'].to_json, s['source_processing_hz'], s['wall_processing_hz'], s.dig('latency_sim_sec', 'min'), s.dig('latency_sim_sec', 'median'), s.dig('latency_sim_sec', 'max'), s.dig('prediction_wall_ms', :median), s.dig('annotations', 'observed_false_positive_boxes'), s['gazebo_robot_motion_m'], s.dig('annotations', 'localization_condition') || 'unseeded']
  end
end
puts "Wrote #{summaries.size} trial summaries to #{out}"
summaries.each do |s|
  puts [s['trial'], s.dig('gazebo_camera', :horizontal_distance_m)&.round(3), s['recognized_images'], s['actual_search_accepted_images'], s['actual_search_accurate_images'], s['distinct_processed_images'], s['confidence'], s['actual_search_rejections']].to_json
end
