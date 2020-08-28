import pickle 
import numpy as np 
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.ensemble import RandomForestClassifier 
from scipy.stats import entropy, multivariate_normal
from copy import deepcopy
from sklearn.cluster import SpectralClustering
from sklearn.feature_selection import VarianceThreshold
from sklearn.svm import SVC
from sklearn.decomposition import PCA 

model_paths = {
    'slope_push': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/SLOPE_PUSH_state_data_GP3_over_edge.txt',
    'push_in_hole': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/PUSH_IN_HOLE_state_data_GP.txt',
    'cross': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/CROSS_state_data_GP.txt',
    'push_towards': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/PUSH_TOWARDS_state_data_GP.txt',
    'reorient': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/REORIENT_state_data_GP2_better.txt',
}

"""analytics_paths = {
    'slope_push': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/SLOPE_PUSH_state_data_analytics_small_box.txt',
    'push_in_hole': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/PUSH_IN_HOLE_state_data_analytics.txt',
    'cross': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/CROSS_state_data_analytics.txt',
    'push_towards': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/PUSH_TOWARDS_state_data_analytics.txt',
    'reorient': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/REORIENT_state_data_analytics.txt',
}"""

analytics_paths = {
    # NOTE: This is what you change to get PLANNER model data training
    'combination': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Pure_Q/Combination_3/Post_Training_Analysis/Stochastic/Pure_MFRL_state_data.txt'
    # /home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Pure_Q/Determ/Combination_1/Pure_MFRL_state_data.txt
}

reward_paths = {
    # 'Pure_Q_Determ': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Elevated_Box_Push_In_Pure_Q_Deterministic/testing_rewards.txt',
    #'Pure_Q_Stochastic': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Pure_Q/Push_In_Hole_1/testing_rewards.txt',
    #'Pure_MPC': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Elevated_Box_Push_In_Pure_MPC/testing_rewards.txt',
    #'Hybrid_Aided': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Elevated_Box_Push_In_Aided_Hybrid/testing_rewards.txt',
    #'Hybrid_Aided_Sparse': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Elevated_Box_Push_In_Aided_Hybrid_2/testing_rewards.txt',  
    #'Pure_MPC_Sparse': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/MPC/Push_In_Hole_One_Hot_1/testing_rewards.txt',
    #'Hybrid_Aided_One_Hot': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Hybrid/Push_In_Hole_One_Hot_2/testing_rewards.txt',  
    #'Pure_Q_Stochastic':'/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Pure_Q/Combination_4/testing_rewards.txt',  
    'MPC':'/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/MPC/Combination_1/testing_rewards.txt',  
    'Pure_Q_Determ': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Pure_Q/Determ/Combination/testing_rewards.txt',
    'Pure_Q_Aided_Gaussian': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Hybrid/Combination_Gaussian_Reverse/testing_rewards.txt',
    'Pure_Q_Aided_GMM': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/Hybrid/Combination_GMM/testing_rewards.txt',
    'Pure_Q_Aided_Model_Q': '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/model_training_data/testing_rewards.txt'
}

features = {
    'bot_to_box_x': 0,
    'bot_to_box_y': 1,
    'bot_to_box_z': 2,
    'box_pitch': 3,
    'box_yaw': 4,
    'bot_to_goal_x': 5,
    'bot_to_goal_y': 6,
    'bot_to_goal_z': 7,
    'bot_yaw': 8  
}

indices_to_features = {
    0:'bot_to_box_x',
    1:'bot_to_box_y',
    2:'bot_to_box_z',
    3:'box_pitch',
    4:'box_yaw',
    5:'bot_to_goal_x',
    6:'bot_to_goal_y',
    7:'bot_to_goal_z',
    8:'bot_yaw',
    9:'moveable_box_indicator'
}

class Analysis(object):
    def __init__(self, mode):
        self.id_to_primitive = {}
        self.primitive_to_id = {}
        self.histogram_bins = 15

        self.min_features_in_boundary_search = 3
        self.samples_for_boundary_search = 10
        self.prob_choose_different_point = .05
        self.min_boundary = .05
        self.min_step_size = .2
        self.n_clusters = 7

        self.mode = mode
        if self.mode == 'Analysis':
            self.curr_path = analytics_paths
        else:
            self.curr_path = model_paths
        self.success, self.success_prime, self.fail, self.fail_prime, self.success_start, self.fail_start, self.time, self.success_rewards, self.fail_rewards = self._process_data()
        pass 
    
    def analyze_requested_data(self):
        """self.analyze_feature_to_time('box_yaw', 'reorient')
        self.analyze_feature_to_time('bot_to_box_y', 'reorient')
        self.analyze_feature_to_time('bot_to_box_y', 'push_in_hole')
        self.analyze_feature_to_time('bot_to_box_x', 'push_in_hole')

        self.analyze_3d_start_state('bot_to_box_x', 'bot_to_box_y', 'box_yaw','reorient')
        self.analyze_3d_start_state('bot_to_box_x', 'bot_to_box_y', 'box_yaw', 'push_in_hole')
        self.analyze_3d_start_state('bot_to_box_x', 'bot_to_box_y', 'bot_yaw', 'cross')
        self.analyze_3d_start_state('bot_to_box_x', 'bot_to_box_y', 'box_yaw', 'push_towards')

        self.analyze_start_state('bot_to_box_y', 'push_in_hole')    
        self.analyze_start_state('box_yaw', 'reorient')
        self.analyze_start_state('bot_to_goal_y', 'push_towards')
        self.analyze_start_state('bot_to_goal_y', 'reorient')
        self.analyze_start_state('bot_yaw', 'slope_push')  """
        # self.analyze_3d_start_state('bot_to_box_y', 'bot_to_goal_x', 'bot_to_goal_y', 'push_in_hole')
        #features = self.random_forest_extract_decision_boundaries(self.success, self.fail, 'push_in_hole',  plot=True)
        #features = self.random_forest_extract_features(self.success_start, self.fail_start, 'cross', plot=True)
        #features = self.random_forest_extract_features(self.success_start, self.fail_start, 'push_towards', plot=True)
        #features = self.random_forest_extract_features(self.success_start, self.fail_start, 'reorient', plot=True)
        #features = self.random_forest_extract_features(self.success_start, self.fail_start, 'slope_push', plot=True)
        # self.plot_past_rewards()
        self.rf, self.svm, self.pdf_generator, points_in_ellipsoid = self.measure_reliability_and_success(self.success['combination'], self.fail['combination'])
        return 

    def plot_past_rewards(self):
        colors = ['r', 'g', 'b', 'k', 'c', 'm', 'y']
        for i, (name, path) in enumerate(reward_paths.items()):
            with open(path, 'rb') as f:
                data = pickle.load(f)
            if len(data) <= 200:
                continue
                #data = data[:200]
            #x = range(len(data))
            # plt.plot(x, data)
            plt.title("Rewards Over Episodes w/ Moving Average")
            #window= np.ones(int(100))/float(100)
            #lineRewards = np.convolve(data, window, 'same')
            lineRewards = self.get_moving_average(data, 100)[40:] # approxiately when starts training
            x = range(len(lineRewards))
            plt.plot(x, lineRewards, colors[i], label=name)
            plt.legend()
            grid = True
        plt.show()
    
    def get_moving_average(self, lst, resolution):
        cumsum, moving_aves = [0], []

        for i, x in enumerate(lst, 1):
            cumsum.append(cumsum[i-1] + x)
            if i>= resolution:
                moving_ave = (cumsum[i] - cumsum[i-resolution])/resolution
                #can do stuff with moving_ave here
                moving_aves.append(moving_ave)
            else:
                moving_aves.append(cumsum[i] / len(cumsum))
        return moving_aves
            
    
    def measure_reliability_and_success(self, success, fail):
        all_points = np.vstack((success, fail))
        indices = self.variance_thresholding(all_points)
        # NOTE: Got rid of variance thresholding here
        indices = np.linspace(0, all_points.shape[1], all_points.shape[1], endpoint=False).astype(int)
        all_points = all_points[:, indices]
        print(success.shape, fail.shape)

        print('  Features kept for analysis: ')
        for index in indices:
            print(indices_to_features[index])
        print('')
        labels = np.hstack((np.repeat(1, success.shape[0]), np.repeat(0, fail.shape[0])))

        pdf_generator = self.fit_gaussian(all_points)
        # filter for similar points in all_points here
        rf, rf_accuracy = self.random_forest_fit(all_points, labels)
        svm, svm_accuracy, svm_parameters = self.polynomial_svm(all_points, labels)
        print(' Random Forest Accuracy: ', rf_accuracy)
        print(' SVM Accuracy: ', svm_accuracy)
        print('')

        priors = np.apply_along_axis(pdf_generator, 1, all_points)
        conditional = rf.predict_proba(all_points)[:, 1] # probability success
        # posterior = np.sum((conditional * priors) / np.sum(priors))
        posterior = np.mean(conditional)
        # print(' Probability of Success Using RF: ', posterior)
        print(' Probability of Success Using MC: ', success.shape[0] / float(success.shape[0] + fail.shape[0]))
        print('')

        mask_for_success = svm.predict(all_points).astype(bool)
        points_in_ellipsoid = all_points[mask_for_success, :]

        assert len(self.success_rewards + self.fail_rewards) == all_points.shape[0]
        rewards_associated_with_points_in_ellipsoid = np.array(self.success_rewards + self.fail_rewards)[mask_for_success]

        funnel_priors = np.apply_along_axis(pdf_generator, 1, points_in_ellipsoid)
        conditional = rf.predict_proba(points_in_ellipsoid)[:, 1]
        # posterior = np.sum((conditional * funnel_priors) / np.sum(funnel_priors))
        posterior = np.mean(conditional)
        approx_prob_in_ellipsoid = np.sum(funnel_priors) / np.sum(priors)
        # print(' Posterior Probability Success Given Inside Ellipsoid Funnel Using RF: ', posterior)
        labels_of_points_in_ellipsoid = labels[mask_for_success]
        print(' Posterior Probability Success Given Inside Ellipsoid Funnel Using MC: ', np.sum(labels_of_points_in_ellipsoid) / float(labels_of_points_in_ellipsoid.shape[0]))
        print(' Prior MC Probability of Being Inside Ellipsoid Funnel: ', np.sum(mask_for_success) / float(mask_for_success.shape[0]))
        print('')

        bounds = self.get_bounds(points_in_ellipsoid)
        print('')
        print(' Hyperellipsoid funnel boundaries: ')
        for i in range(len(bounds)):
            feature_name = indices_to_features[indices[i]]
            boundary = bounds[i]
            print(feature_name, ' bounds: ', boundary)
        print('')

        pca = PCA(n_components=3)
        pca.fit(points_in_ellipsoid)
        transformed_points = pca.transform(points_in_ellipsoid)

        s1 = transformed_points[:, 0]
        s2 = transformed_points[:, 1]
        s3 = transformed_points[:, 2]

        minimum_reward = np.min(rewards_associated_with_points_in_ellipsoid)
        normalized_rewards = (rewards_associated_with_points_in_ellipsoid - minimum_reward)
        normalized_rewards = normalized_rewards / np.max(normalized_rewards)
        colors = np.vstack([np.array([[0, r, 0]]) for r in normalized_rewards])

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(s1, s2, s3, c=colors , marker='o')

        # ALL POINTS
        transformed_points = pca.transform(all_points)

        s1 = transformed_points[:, 0]
        s2 = transformed_points[:, 1]
        s3 = transformed_points[:, 2]

        all_rewards = self.success_rewards + self.fail_rewards
        minimum_reward = np.min(all_rewards)
        normalized_rewards = (all_rewards - minimum_reward)
        normalized_rewards = normalized_rewards / np.max(normalized_rewards)
        colors = np.vstack([np.array([[0, r, 0]]) for r in normalized_rewards])

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(s1, s2, s3, c=colors , marker='o')
        plt.show()
        
        return rf, svm, pdf_generator, points_in_ellipsoid
    
    def get_bounds(self, points_in_ellipsoid):
        # Returns list of tuples each corresponding to limits of the respective index
        boundaries = []
        for i in range(points_in_ellipsoid.shape[1]):
            minimum = np.min(points_in_ellipsoid[:, i])
            maximum = np.max(points_in_ellipsoid[:, i])
            boundaries.append((minimum, maximum))
        return boundaries
    
    def variance_thresholding(self, dataset):
        selector = VarianceThreshold(threshold = .02)
        selector.fit(dataset)
        return selector.get_support(indices=True)
    
    def fit_gaussian(self, dataset):
        """ Assumes dataset is shape (num points, num features)"""
        mean = np.mean(dataset, axis=0)
        covariance = np.cov(dataset.T) # takes in shape (num features, num points)
        return lambda x: multivariate_normal.pdf(x, mean=mean, cov=covariance)
    
    def polynomial_svm(self, dataset, labels):
        svm = SVC(kernel='poly', degree=2, gamma='scale', C=1000)
        svm.fit(dataset, labels)
        accuracy = svm.score(dataset, labels)
        return svm, accuracy, svm.get_params(deep=False)
    
    def random_forest_fit(self, points, labels):
        rf = RandomForestClassifier(n_estimators=100, max_depth = 4)
        rf.fit(points, labels)
        accuracy = rf.score(points, labels)
        return rf, accuracy
    
    def random_forest_extract_decision_boundaries(self, success, fail, primitive, plot):
        points = np.vstack((success[primitive], fail[primitive]))
        labels = np.hstack((np.repeat(1, success[primitive].shape[0]), np.repeat(0, fail[primitive].shape[0])))
        rf, filtered_points, feature_indices  = self.random_forest_extract_features(points, labels, primitive, plot, filter_features=True)
        boundary_points = self.iteratively_find_boundary_points(rf, filtered_points.shape[1])
        clusters = self.cluster_boundary_points(boundary_points, feature_indices, primitive)
    
    def random_forest_extract_features(self, points, labels, primitive, plot, filter_features=False):
        rf, accuracy = self.random_forest_fit(points, labels)
        features = rf.feature_importances_
        feature_indices = [i for i in range(features.size)]
        if plot:
            plt.bar(np.array(range(features.size)), features)
            plt.title(' Importance of feature for ' + primitive)
            plt.show()

        if filter_features:
            highest_prop = np.max(features)
            default_include = sorted([(x,i) for (i,x) in enumerate(features)], reverse=True)[:self.min_features_in_boundary_search]
            feature_indices = [i for i,x in enumerate(features) if features[i] > highest_prop / 3 or (x,i) in default_include]
            points = points[:, feature_indices]
            rf = RandomForestClassifier(n_estimators=500, max_depth=2)
            rf.fit(points, labels)
        return rf, points, feature_indices
    
    def iteratively_find_boundary_points(self, rf, feature_size):
        boundary = []

        curr_state = np.zeros(feature_size)
        delta = 0

        while len(boundary) < 1000:
            curr_prob = rf.predict_proba(curr_state.reshape(1,-1))
            if abs(.5 - np.asscalar(curr_prob[:, 0])) < self.min_boundary:
                boundary.append(curr_state)
            i = np.random.random()
            if i < self.prob_choose_different_point or delta == 0:
                curr_state = np.random.uniform(-3, 3, size=feature_size)
            
            samples = np.random.normal(loc=curr_state.reshape(1,-1), scale=1, size=(self.samples_for_boundary_search, curr_state.size))
            next_z = np.abs(.5 - rf.predict_proba(samples)[:, 0]) 
            curr_z = np.abs(.5 - curr_prob[:, 0])

            delta = curr_z - np.min(next_z)
            print(curr_prob, delta, len(boundary))
            idx = np.argmin(next_z)
            direction = samples[idx, :] - curr_state 
            curr_state = curr_state + direction * (max(delta, self.min_step_size))

        return np.vstack(boundary)
    
    def cluster_boundary_points(self, boundary_points, feature_indices, primitive):
        clustering = SpectralClustering(n_clusters = self.n_clusters, random_state = 0).fit(boundary_points)
        labels = np.array(clustering.labels_)

        # Test: we assume size 3
        s1 = boundary_points[:, 0].ravel()
        s2 = boundary_points[:, 1].ravel()
        s3 = boundary_points[:, 2].ravel()

        colors = np.array(['r', 'g','b', 'c', 'm', 'y', 'k'])

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(s1, s2, s3, c=colors[labels], marker='o')
        ax.set_xlabel(indices_to_features[feature_indices[0]])
        ax.set_ylabel(indices_to_features[feature_indices[1]])
        ax.set_zlabel(indices_to_features[feature_indices[2]])
        plt.title('Decision boundary for ' + primitive)
        plt.show()
        return
    
    def _process_data(self):
        # Return successful and failures for each of the primitives
        success = {}
        success_prime = {}
        fail = {}
        fail_prime = {}

        success_start = {}
        fail_start = {}

        total_time = {}

        success_rewards = []
        fail_rewards = []
        for i, key in enumerate(self.curr_path.keys()):
            path = self.curr_path[key]
            with open(path, "rb") as fp:
                data = pickle.load(fp)
            self.id_to_primitive[i] = key
            self.primitive_to_id[key] = i
            for rollout in data:
                if type(rollout[-1]) == int:
                    achieved = rollout[-1]
                    states = rollout[:-1]
                    result = states[-1]
                elif type(rollout[-2]) == int:
                    achieved = rollout[-2]
                    time = rollout[-1]
                    states = rollout[:-2]
                    result = states[-1]
                    if achieved:
                        curr_time = total_time.get(key, [])
                        curr_time.append(time)
                        total_time[key] = curr_time
                else:
                    continue

                if achieved:
                    curr_success = success.get(key, np.zeros((1, states[0].size)))
                    curr_prime = success_prime.get(key, np.zeros((1, states[0].size)))
                    success[key] = np.vstack((curr_success, np.vstack(states)))
                    results = np.vstack([result for i in range(len(states))])
                    success_prime[key] = np.vstack((curr_prime, results))

                    curr_success = success_start.get(key, np.zeros((1, states[0].size)))
                    success_start[key] = np.vstack((curr_success, states[0]))

                    success_rewards.extend(self.get_accumulated_rewards_success(len(states)))
                else:
                    curr_fail = fail.get(key, np.zeros((1, states[0].size)))
                    curr_prime = fail_prime.get(key, np.zeros((1, states[0].size)))
                    fail[key] = np.vstack((curr_fail, states))
                    results = np.vstack([result for i in range(len(states))])
                    fail_prime[key] = np.vstack((curr_prime, results))

                    curr_fail = fail_start.get(key, np.zeros((1, states[0].size)))
                    fail_start[key] = np.vstack((curr_fail, states[0]))
                    fail_rewards.extend(self.get_accumulated_rewards_fail(len(states)))
        for key in self.curr_path.keys():
            success[key] = success[key][1:, :]
            success_prime[key] = success_prime[key][1:, :]
            fail[key] = fail[key][1:, :]
            fail_prime[key] = fail_prime[key][1:, :]
            success_start[key] = success_start[key][1:, :]
            fail_start[key] = fail_start[key][1:, :]
        return success, success_prime, fail, fail_prime, success_start, fail_start, total_time, success_rewards, fail_rewards
    
    def get_accumulated_rewards_success(self, length):
        return [5 - (i * .1) for i in range(length)]
    
    def get_accumulated_rewards_fail(self, length):
        return [-3 - (i * .1) for i in range(length)]
    
    def get_data(self):
        return self.success, self.success_prime, self.fail, self.fail_prime, self.id_to_primitive, self.primitive_to_id
    
    def get_start_data(self):
        return self.success_start, self.fail_start
        
    def analyze_start_state(self, feature, primitive):
        """
        Takes a dataset (consisting of dictionary mapping primitives to data) and analyzes distribution of specific feature
        """
        s = self.success_start[primitive]
        f = self.fail_start[primitive]

        index = features[feature]
        s = s[:, index:index+1]
        f = f[:, index:index+1]

        # computing bin properties 
        num_bin = self.histogram_bins
        start = min(np.min(s), np.min(f))
        end = max(np.max(s), np.max(f))
        bin_lims = np.linspace(start,end,num_bin+1)
        bin_centers = 0.5*(bin_lims[:-1]+bin_lims[1:])
        bin_widths = bin_lims[1:]-bin_lims[:-1]
        
        # computing histograms
        s_hist, _ = np.histogram(s, bins=bin_lims)
        f_hist, _ = np.histogram(f, bins=bin_lims)

        # normalizing
        norm = (s_hist + f_hist).astype(float)
        s_hist = s_hist/norm
        f_hist = f_hist/norm

        plt.bar(bin_centers, s_hist, width = bin_widths, align = 'center', alpha=.5)
        plt.bar(bin_centers, f_hist, width = bin_widths, align = 'center', alpha=.5)
        plt.title('Normalized Distribution of Starting Feature ' + feature.upper() + ' for ' + primitive.upper() + ' Primitive')
        plt.show()
    
    def analyze_2d_start_state(self, feature1, feature2, primitive):
        """
        Takes a dataset and analyzes two features' success and failure initial states for a given primitive
        """
        s = self.success_start[primitive]
        f = self.fail_start[primitive]

        index1 = features[feature1]
        index2 = features[feature2]

        s1 = s[:, index1: index1+1]
        s2 = s[:, index2: index2+1]

        f1 = f[:, index1: index1+1]
        f2 = f[:, index2: index2+1]

        colors = np.repeat(np.array([[0, 255, 0]]), s1.shape[0], axis=0)
        colors = np.vstack((colors, np.repeat(np.array([[255, 0, 0]]), f1.shape[0], axis = 0)))

        s1 = np.vstack((s1, f1)).ravel()
        s2 = np.vstack((s2, f2)).ravel()
        plt.scatter(s1, s2, c=colors/255)
        plt.xlabel(feature1)
        plt.ylabel(feature2)
        plt.title(' Successes and Failures for Starting Features in ' + primitive + ' Primitive')
        plt.show()
    
    def analyze_3d_start_state(self, feature1, feature2, feature3, primitive):
        s = self.success_start[primitive]
        f = self.fail_start[primitive]

        index1 = features[feature1]
        index2 = features[feature2]
        index3 = features[feature3]

        s1 = s[:, index1: index1+1]
        s2 = s[:, index2: index2+1]
        s3 = s[:, index3: index3+1]

        f1 = f[:, index1: index1+1]
        f2 = f[:, index2: index2+1]
        f3 = f[:, index3: index3+1]

        colors = np.repeat(np.array([[0, 255, 0]]), s1.shape[0], axis=0)
        colors = np.vstack((colors, np.repeat(np.array([[255, 0, 0]]), f1.shape[0], axis = 0)))

        s1 = np.vstack((s1, f1)).ravel()
        s2 = np.vstack((s2, f2)).ravel()
        s3 = np.vstack((s3, f3)).ravel()

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(s1, s2, s3, c=colors/255 , marker='o')

        ax.set_xlabel(feature1)
        ax.set_ylabel(feature2)
        ax.set_zlabel(feature3)
        plt.title(' Successes and Failures for Starting Features in ' + primitive + ' Primitive')
        plt.show()

    def analyze_feature_to_time(self, feature, primitive):
        """
        Takes a feature and plots a primitive's time for completion (assumed to be successful) with respect to initial feature condition
        """
        time = np.array(self.time[primitive])
        s = self.success_start[primitive]
        s = s[:, features[feature]].ravel()
    
        plt.scatter(s, time)
        plt.xlabel(feature)
        plt.ylabel('Time in seconds')
        plt.title('Time Distribution of Starting Feature ' + feature + ' in Successful Rollouts for ' + primitive + 'Primitive')
        plt.show()

    def analyze_robustness_to_properties(self, property, primitive):
        """
        Takes a property (size of box, size of robot, depth of hole) and plots a primitive's success rate
        """
        
    