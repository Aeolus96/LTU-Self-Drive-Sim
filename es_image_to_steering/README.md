# es_image_to_steering

The es_image_to_steering package uses Keras with a PyTorch backend to predic steering angles (Twist msg) from input ROS images.

This package uses models trained with Evolutionary Strategy. The included pre-trained model is the smallest model due to size constraints. For bigger models, you can contact the developer for pre-trained models or instead use your own keras-compatible model.
