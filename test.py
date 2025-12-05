########################### Code starts here ###########################

        occupied_mask = np.where(self.occupancy.probs >= 0.5, 1, 0)
        unknown_mask = np.where(self.occupancy.probs == -1, 1, 0)
        unoccupied_mask = np.where((self.occupancy.probs < 0.5) & (self.occupancy.probs>=0), 1, 0)

        kernel = np.ones((window_size, window_size)) / window_size**2
        occupied=convolve2d(occupied_mask, kernel, mode=‘same’)
        unoccupied=convolve2d(unoccupied_mask, kernel, mode=‘same’)
        unknown=convolve2d(unknown_mask, kernel, mode=‘same’)

        frontier_mask = np.where((occupied==0) & (unoccupied>=0.3) & (unknown>=0.2),1,0)
        frontier_states = np.transpose(np.nonzero(np.transpose(frontier_mask)))
        frontier_states = self.occupancy.grid2state(frontier_states)

        if len(frontier_states) == 0:
            print(“Finished exploring”)

        frontier_state  = np.argmin(np.linalg.norm(frontier_states-current_state,axis=1))
        frontier_state = frontier_states[frontier_state,:]
        ########################### Code ends here ###########################
