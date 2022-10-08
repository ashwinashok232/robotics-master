# Natural Language Processing

## Description
Given a dataset of movie titles, plots and genres. Creates a machine learning model that 
uses NLP techniques for multi-class movie genre prediction using the plot synopsis.

## Method
Pre-processing - Lowercasing, removing all non-alpha-numeric characters (using regular expressions), stopword 
removal, stemming. The genre column of the training data was binarized into a one-hot array.

ML Model - Used tf-idf (term frequency inverse document frequncy) to use the weightages of words as features
for classification. Then, using logistic regression with a OneVsRest multi-class classifier for classification.
I also attempted to use Gaussian Naive Bayes for classification, but it was too computationally expensive.

## Results
The maximum F1 score obtained was ~0.51. Given that there were 20 possible genres, the training data needed to
be much more extensive for better prediction.

